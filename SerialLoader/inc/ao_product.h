/*
 * ao_product.h
 *
 *  Created on: Apr 30, 2020
 *      Author: bfisher
 */

#ifndef AO_PRODUCT_H_
#define AO_PRODUCT_H_

/* iManufacturer */
#define AO_iManufacturer_LEN 0x20
#define AO_iManufacturer_STRING "altusmetrum.org"
#define AO_iManufacturer_UCS2 'a', 0, 'l', 0, 't', 0, 'u', 0, 's', 0, 'm', 0, 'e', 0, 't', 0, 'r', 0, 'u', 0, 'm', 0, '.', 0, 'o', 0, 'r', 0, 'g', 0

/* iProduct */
#define AO_iProduct_LEN 0x16
#define AO_iProduct_STRING "AltosFlash"
#define AO_iProduct_UCS2 'A', 0, 'l', 0, 't', 0, 'o', 0, 's', 0, 'F', 0, 'l', 0, 'a', 0, 's', 0, 'h', 0

/* iSerial */
#define AO_iSerial_LEN 0x0e
#define AO_iSerial_STRING "000001"
#define AO_iSerial_UCS2 '0', 0, '0', 0, '0', 0, '0', 0, '0', 0, '1', 0

/* iSerial */
#define AO_iSerial_NUMBER 1

/* idProduct */
#define AO_idProduct_NUMBER 0x000a

/* idVendor */
#define AO_idVendor_NUMBER 0xfffe

/* iVersion */
#define AO_iVersion_STRING "1.9.2"

#endif /* AO_PRODUCT_H_ */
