#include <ecrt.h>
#include <stdio.h>
#include <time.h>
#include <stdint.h>
#include <sched.h>
#include <unistd.h>
#include <sys/mman.h>
#include <signal.h>

#define PERIOD_NS 1000000 // 1ms, 1MHz
#define TIMESPEC2NS(T) ((uint64_t) (T.tv_sec) * 1000000000LL + (uint64_t) (T.tv_nsec))

ec_slave_config_t *drive[2]; // Assumed 2 slaves
ec_domain_t *domain1;
uint8_t *domain1_pd;

struct timespec wakeupTime;
ec_master_t *master;

void signal_handler(int sig)
{
    printf("Signal %d received. Exiting...\n", sig);
    // Clean up here
    exit(0);
}

void configure_slave(ec_master_t *master, unsigned int alias, unsigned int position, uint32_t vendor_id, uint32_t product_code, int slave_index)
{
    // Request slave configuration
    drive[slave_index] = ecrt_master_slave_config(master, alias, position, vendor_id, product_code);
    if (!drive[slave_index])
    {
        printf("Failed to get slave configuration for position %d\n", position);
        return;
    }

    // PDO mapping
    ec_pdo_entry_info_t slave_pdo_entries[] = {
        {0x6040, 0x00, 16}, /* Controlword */
        {0x607a, 0x00, 32}, /* Target Position */
        {0x60FF, 0x00, 32}, /* Target Velocity */
        {0x6071, 0x00, 16}, /* Target Torque */
        {0x6041, 0x00, 16}, /* Statusword */
        {0x6064, 0x00, 32}, /* Position Actual Value */
        {0x606c, 0x00, 32}, /* Velocity Actual Value */
        {0x6077, 0x00, 16}, /* Torque Actual Value */
    };

    ec_pdo_info_t slave_pdos[] = {
        {0x1600, 4, slave_pdo_entries + 0}, /* 2nd Receive PDO Mapping */
        {0x1a00, 4, slave_pdo_entries + 4}, /* 2nd Transmit PDO Mapping */
    };

    ec_sync_info_t slave_syncs[] = {
        {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
        {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 1, slave_pdos + 0, EC_WD_ENABLE},
        {3, EC_DIR_INPUT, 1, slave_pdos + 1, EC_WD_DISABLE},
        {0xFF}
    };

    // Configure slave PDO
    if (ecrt_slave_config_pdos(drive[slave_index], EC_END, slave_syncs))
    {
        printf("Failed to configure PDOs for position %d\n", position);
    }

    // Enable Distributed Clocks (DC)
 ecrt_slave_config_dc(drive[slave_index], 0x0300, PERIOD_NS, 0, 0, 0);

}

uint controlword, statusword ,
     target_position,actual_position,
     target_velocity,actual_velocity,
     target_torque,actual_torque;

void configure_domain(ec_master_t *master, uint32_t vendor_id, uint32_t product_code)
{

         ec_pdo_entry_reg_t domain1_regs[] = {
        {0, 0, vendor_id, product_code, 0x6040, 0x00, &controlword},
        {0, 0, vendor_id, product_code, 0x607a, 0x00, &target_position},
        {0, 0, vendor_id, product_code, 0x60FF, 0x00, &target_velocity},
        {0, 0, vendor_id, product_code, 0x6071, 0x00, &target_torque},
        {0, 0, vendor_id, product_code, 0x6041, 0x00, &statusword},
        {0, 0, vendor_id, product_code, 0x6064, 0x00, &actual_position},
        {0, 0, vendor_id, product_code, 0x606c, 0x00, &actual_velocity},
        {0, 0, vendor_id, product_code, 0x6077, 0x00, &actual_torque},
        {}
    };

    // Create Process Data Domain
    domain1 = ecrt_master_create_domain(master);
    if (!domain1)
    {
        printf("Failed to create domain\n");
        return;
    }

    // Register PDO entries
    if (ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs))
    {
        printf("PDO entry registration failed\n");
        return;
    }
}

void drive_multiple_slaves(int num_slaves)
{
    int32_t actPos[2], targetPos[2];

    while (1)
    {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeupTime, NULL);

        ecrt_master_receive(master);
        ecrt_domain_process(domain1);

        for (int i = 0; i < num_slaves; ++i)
        {
            // Read actual positions from slaves
            actPos[i] = EC_READ_S32(domain1_pd + actual_position);
            targetPos[i] = actPos[i] + 0;  // For example, target position is actual position + 5000
            EC_WRITE_S32(domain1_pd + target_position, targetPos[i]);

            // Print actual positions
            printf("Slave %d Actual Position: %d, Target Position: %d\n", i, actPos[i], targetPos[i]);
        }

        ecrt_domain_queue(domain1);
        ecrt_master_send(master);
    }
}

int main()
{

    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
        perror("sched_setscheduler failed\n");
    }

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        printf("mlockall failed\n");
        return -1;
    }

    signal(SIGINT, signal_handler);

    // Request EtherCAT master
    master = ecrt_request_master(0);
    if (!master)
    {
        printf("Requesting master failed\n");
        return -1;
    }

    uint32_t vendor_id = 0x5a65726f;
    uint32_t product_code = 0x00029252;

    // Configure slaves
    configure_slave(master, 0, 0, vendor_id, product_code, 0);
    configure_slave(master, 0, 1, vendor_id, product_code, 1);  // Second slave

    // Configure domain
    configure_domain(master, vendor_id, product_code);

    // Set reference clock (first slave as reference)
    if (ecrt_master_select_reference_clock(master, drive[0]))
    {
        printf("Failed to select reference clock!\n");
        return -1;
    }

    // Activate master
    if (ecrt_master_activate(master))
    {
        return -1;
    }

    // Get domain data pointer
    domain1_pd = ecrt_domain_data(domain1);
    if (!domain1_pd)
    {
        return -1;
    }

    // Initialize application time for Distributed Clocks (DC)
    struct timespec masterInitTime;
    clock_gettime(CLOCK_MONOTONIC, &masterInitTime);
    ecrt_master_application_time(master, TIMESPEC2NS(masterInitTime));

    clock_gettime(CLOCK_MONOTONIC, &wakeupTime);

    // Drive multiple slaves
    drive_multiple_slaves(2);  // Assumed 2 slaves

    return 0;
}
