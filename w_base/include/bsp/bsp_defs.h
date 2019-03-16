/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */
#ifndef H_BSP_DEFS_H
#define H_BSP_DEFS_H


#ifdef __cplusplus
extern "C" {
#endif

//LORA

/*
#define SX1272_DIO_0 	MCU_GPIO_PORTB(1)	 //ok
#define SX1272_DIO_1 	MCU_GPIO_PORTB(10)	 //ok
#define SX1272_DIO_2 	MCU_GPIO_PORTB(11)	 //ok
#define SX1272_DIO_3 	MCU_GPIO_PORTB(7)	 //ok
#define SX1272_DIO_4 	MCU_GPIO_PORTB(5)	 //ok
#define SX1272_DIO_5 	MCU_GPIO_PORTB(4)	 //ok
#define SX1272_NSS 	    MCU_GPIO_PORTB(0)	 //ok
#define SX1272_RESET 	MCU_GPIO_PORTA(2)	 //ok
*/


//SPI
#define RADIO_MOSI MCU_GPIO_PORTA(7);
#define RADIO_MISO MCU_GPIO_PORTA(6);
#define RADIO_SCLK MCU_GPIO_PORTA(5);



//#define BSP_UART_TX MCU_GPIO_PORTA(9)
//#define BSP_UART_RX MCU_GPIO_PORTA(10)

#ifdef __cplusplus
}
#endif

#endif  /* H_BSP_DEFS_H */
