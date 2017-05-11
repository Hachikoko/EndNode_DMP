#include "stm32f4xx.h"
//#include "usart.h"
#include "delay.h"
#include "Timer.h"
#include "link_queue.h"

#include "uart.h"
#include "i2c.h"
#include "gpio.h"
#include "main.h"
#include "board_init.h"
#include "parameter.h"
#include "NRF24L01.h"

#include "Debug_switch.h"




u8 node_index_for_base_station = 0; 	//分配节点号
u8 node_index_for_end_node = 6;  		//绝对节点编号
u8 extra_node_flag = 0;
u8 current_frequency = 0;
Link_Queue * ptr_link_queue;
char ret_words[100];



int main(void)
{
	inv_error_t result;
    unsigned char accel_fsr,  new_temp = 0;
    unsigned short gyro_rate, gyro_fsr;
    unsigned long timestamp;
    struct int_param_s int_param;
	unsigned char new_compass = 0;
    unsigned short compass_fsr;
//	int a = 10000;
	
	ptr_link_queue = creat_link_queue();
	
	//滴答定时器，GPIO外部中断，IIC、串口1初始化
	board_init(); 
	//初始化MUP9250
	result = mpu_init(&int_param);
    if (result) {
		MPL_LOGE("Could not initialize gyro.\r\n");
	}
	
	//初始化MPL库
	result = inv_init_mpl();
	if (result) {
		MPL_LOGE("Could not initialize MPL.\r\n");
	}
	//NRF24L01初始化
	NRF24L01_Init();
	while(NRF24L01_Check()){
		delay_ms(100);
	}
	current_frequency = node_index_for_end_node * 5;
	RX_Mode();
	
	/* Compute 6-axis and 9-axis quaternions. */
    inv_enable_quaternion();
//    inv_enable_9x_sensor_fusion();
	
	// Update gyro biases when not in motion.
    inv_enable_fast_nomot();

    /* Update gyro biases when temperature changes. */
    inv_enable_gyro_tc();
	
	#ifdef COMPASS_ENABLED
    /* Compass calibration algorithms. */
    inv_enable_vector_compass_cal();
    inv_enable_magnetic_disturbance();
	#endif
	
	/* If you need to estimate your heading before the compass is calibrated,
    * enable this algorithm. It becomes useless after a good figure-eight is
    * detected, so we'll just leave it out to save memory.	
    */
	
	inv_enable_heading_from_gyro();	
	
	/* Allows use of the MPL APIs in read_from_mpl. */
    inv_enable_eMPL_outputs();
	
	result = inv_start_mpl();
	if (result == INV_ERROR_NOT_AUTHORIZED) {
		while (1) {
			MPL_LOGE("Not authorized.\n");
		}
	}
	if (result) {
		MPL_LOGE("Could not start the MPL.\n");
	}
	
	/* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
	#ifdef COMPASS_ENABLED
		mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
	#else
		mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
	#endif
	
	/* Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);
	
	#ifdef COMPASS_ENABLED
    /* The compass sampling rate can be less than the gyro/accel sampling rate.
     * Use this function for proper power management.
     */
		mpu_set_compass_sample_rate(100 / COMPASS_READ_MS);
	#endif
	
	/* Read back configuration in case it was set improperly. */
	mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
	
	#ifdef COMPASS_ENABLED
    mpu_get_compass_fsr(&compass_fsr);
	#endif
	
	/* Sync driver configuration with MPL. */
    /* Sample rate expected in microseconds. */
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);
	
	#ifdef COMPASS_ENABLED
    /* The compass rate is independent of the gyro and accel rates. As long as
     * inv_set_compass_sample_rate is called with the correct value, the 9-axis
     * fusion algorithm's compass correction gain will work properly.
     */
    inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);
	#endif
	
	    /* Set chip-to-body orientation matrix.
     * Set hardware units to dps/g's/degrees scaling factor.
     */
    inv_set_gyro_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)gyro_fsr<<15);
    inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
            (long)accel_fsr<<15);
	#ifdef COMPASS_ENABLED
    inv_set_compass_orientation_and_scale(
            inv_orientation_matrix_to_scalar(compass_pdata.orientation),
            (long)compass_fsr<<15);
	#endif
	
	/* Initialize HAL state variables. */
	#ifdef COMPASS_ENABLED
		hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
	#else
		hal.sensors = ACCEL_ON | GYRO_ON;
	#endif
		hal.dmp_on = 0;
		hal.report = 0;
		hal.rx.cmd = 0;
		hal.next_pedo_ms = 0;
		hal.next_compass_ms = 0;
		hal.next_temp_ms = 0;
		
	/* Compass reads are handled by scheduler. */
	get_tick_count(&timestamp);
	
	/* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
    dmp_load_motion_driver_firmware();  //加载DMP映像
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation)); //设置方位
    dmp_register_tap_cb(tap_cb);
    dmp_register_android_orient_cb(android_orient_cb);  
	
	    /*
     * Known Bug -
     * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
     * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
     * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
     * there will be a 25Hz interrupt from the MPU device.
     *
     * There is a known issue in which if you do not enable DMP_FEATURE_TAP
     * then the interrupts will be at 200Hz even if fifo rate
     * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
     *
     * DMP sensor fusion works only with gyro at +-2000dps and accel +-2G   
     */
//    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
//        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
//        DMP_FEATURE_GYRO_CAL;
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP 
         | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);
    hal.dmp_on = 1;
	
	while(1){
		
	    unsigned long sensor_timestamp;
		int new_data = 0;
		if (USART_GetITStatus(USART2, USART_IT_RXNE)) {
        /* A byte has been received via USART. See handle_input for a list of
         * valid commands.
         */
			USART_ClearITPendingBit(USART2, USART_IT_RXNE);
			handle_input();
		}
		
		get_tick_count(&timestamp);
		#ifdef COMPASS_ENABLED
        /* We're not using a data ready interrupt for the compass, so we'll
         * make our compass reads timer-based instead.
         */
			if ((timestamp > hal.next_compass_ms) && !hal.lp_accel_mode &&
				hal.new_gyro && (hal.sensors & COMPASS_ON)) {
				hal.next_compass_ms = timestamp + COMPASS_READ_MS;			
				new_compass = 1;
			}
		#endif
			
		/* Temperature data doesn't need to be read with every gyro sample.
         * Let's make them timer-based like the compass reads.
         */
        if (timestamp > hal.next_temp_ms) {
            hal.next_temp_ms = timestamp + TEMP_READ_MS;		
            new_temp = 1;
			
        }
		
//		if (hal.motion_int_mode) {
//			
//			/* Enable motion interrupt. */
//			mpu_lp_motion_interrupt(500, 1, 5);						//运行不到
//			/* Notify the MPL that contiguity was broken. */
//			inv_accel_was_turned_off();
//			inv_gyro_was_turned_off();
//			inv_compass_was_turned_off();
//			inv_quaternion_sensor_was_turned_off();
//			/* Wait for the MPU interrupt. */
//			while (!hal.new_gyro) {}
//			/* Restore the previous sensor configuration. */
//			mpu_lp_motion_interrupt(0, 0, 0);
//			hal.motion_int_mode = 0;
//		}
		
		if (!hal.sensors || !hal.new_gyro) {
			continue;
		} 
		
		if (hal.new_gyro && hal.lp_accel_mode) {
            short accel_short[3];
            long accel[3];
            mpu_get_accel_reg(accel_short, &sensor_timestamp);
            accel[0] = (long)accel_short[0];
            accel[1] = (long)accel_short[1];
            accel[2] = (long)accel_short[2];
            inv_build_accel(accel, 0, sensor_timestamp);
            new_data = 1;
            hal.new_gyro = 0;
        } else if (hal.new_gyro && hal.dmp_on) {
            short gyro[3], accel_short[3], sensors;
            unsigned char more;
            long accel[3], quat[4], temperature;
            /* This function gets new data from the FIFO when the DMP is in
             * use. The FIFO can contain any combination of gyro, accel,
             * quaternion, and gesture data. The sensors parameter tells the
             * caller which data fields were actually populated with new data.
             * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
             * the FIFO isn't being filled with accel data.
             * The driver parses the gesture data to determine if a gesture
             * event has occurred; on an event, the application will be notified
             * via a callback (assuming that a callback function was properly
             * registered). The more parameter is non-zero if there are
             * leftover packets in the FIFO.
             */
            dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);
            if (!more)
                hal.new_gyro = 0;
            if (sensors & INV_XYZ_GYRO) {
                /* Push the new data to the MPL. */
                inv_build_gyro(gyro, sensor_timestamp);
                new_data = 1;
                if (new_temp) {
                    new_temp = 0;
                    /* Temperature only used for gyro temp comp. */
                    mpu_get_temperature(&temperature, &sensor_timestamp);
                    inv_build_temp(temperature, sensor_timestamp);
                }
            }
            if (sensors & INV_XYZ_ACCEL) {
                accel[0] = (long)accel_short[0];
                accel[1] = (long)accel_short[1];
                accel[2] = (long)accel_short[2];
                inv_build_accel(accel, 0, sensor_timestamp);
                new_data = 1;
            }
            if (sensors & INV_WXYZ_QUAT) {
                inv_build_quat(quat, 0, sensor_timestamp);
                new_data = 1;
            }
        } else if (hal.new_gyro) {
            short gyro[3], accel_short[3];
            unsigned char sensors, more;
            long accel[3], temperature;
            /* This function gets new data from the FIFO. The FIFO can contain
             * gyro, accel, both, or neither. The sensors parameter tells the
             * caller which data fields were actually populated with new data.
             * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
             * being filled with accel data. The more parameter is non-zero if
             * there are leftover packets in the FIFO. The HAL can use this
             * information to increase the frequency at which this function is
             * called.
             */
            hal.new_gyro = 0;
            mpu_read_fifo(gyro, accel_short, &sensor_timestamp,
                &sensors, &more);
            if (more)
                hal.new_gyro = 1;
            if (sensors & INV_XYZ_GYRO) {
                /* Push the new data to the MPL. */
                inv_build_gyro(gyro, sensor_timestamp);
                new_data = 1;
                if (new_temp) {
                    new_temp = 0;
                    /* Temperature only used for gyro temp comp. */
                    mpu_get_temperature(&temperature, &sensor_timestamp);
                    inv_build_temp(temperature, sensor_timestamp);
                }
            }
            if (sensors & INV_XYZ_ACCEL) {
                accel[0] = (long)accel_short[0];
                accel[1] = (long)accel_short[1];
                accel[2] = (long)accel_short[2];
                inv_build_accel(accel, 0, sensor_timestamp);
                new_data = 1;
            }
        }
		
		#ifdef COMPASS_ENABLED
        if (new_compass) {
            short compass_short[3];
            long compass[3];
            new_compass = 0;
            /* For any MPU device with an AKM on the auxiliary I2C bus, the raw
             * magnetometer registers are copied to special gyro registers.
             */
            if (!mpu_get_compass_reg(compass_short, &sensor_timestamp)) {
                compass[0] = (long)compass_short[0];
                compass[1] = (long)compass_short[1];
                compass[2] = (long)compass_short[2];
                /* NOTE: If using a third-party compass calibration library,
                 * pass in the compass data in uT * 2^16 and set the second
                 * parameter to INV_CALIBRATED | acc, where acc is the
                 * accuracy from 0 to 3.
                 */
                inv_build_compass(compass, 0, sensor_timestamp);
            }
            new_data = 1;
        }
		#endif
		
		if (new_data) {
			
            inv_execute_on_data();
            /* This function reads bias-compensated sensor data and sensor
             * fusion outputs from the MPL. The outputs are formatted as seen
             * in eMPL_outputs.c. This function only needs to be called at the
             * rate requested by the host.
             */
            read_from_mpl();
			//Uart_send('A');
        }
		
		
	}
	
}
