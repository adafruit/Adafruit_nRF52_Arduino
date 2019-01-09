/*
 * random.h
 *
 *  Created on: Jun 10, 2015
 *      Author: tim
 */

#ifndef HOMEKIT_RANDOM_H_
#define HOMEKIT_RANDOM_H_

extern void random_create(uint8_t* p_result, uint8_t length);
extern void randombytes(uint8_t* p_result, uint64_t length);

#endif /* HOMEKIT_RANDOM_H_ */
