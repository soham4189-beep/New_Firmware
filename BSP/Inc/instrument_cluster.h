/*
 * instrument_cluster.h
 *
 *  Created on: 24-Jul-2025
 *      Author: 91626
 */

#ifndef INC_INSTRUMENT_CLUSTER_H_
#define INC_INSTRUMENT_CLUSTER_H_

void split_odometer(void);
void IC_update(void);
#define SWAP_TOTAL_SECONDS  (5U * 60U + 20U)   // 320 seconds
#endif /* INC_INSTRUMENT_CLUSTER_H_ */
