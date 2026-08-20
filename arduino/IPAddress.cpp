/* Copied verbatim from github.com/newdigate/Ethernet src/ (clean-room MIT,
 * see that repo). Per-library duplication is this tree's pattern (NativeEthernet
 * carries its own copy too); consequence: Ethernet and M2Radio's arduino/
 * cannot be imported into one sketch (duplicate class Client). */
/* IPAddress.cpp - IPv4 address value type for the Arduino networking API.
 *
 * Clean-room MIT implementation: written from the documented Arduino API
 * surface and the MIT IPAddress.h declarations, not derived from the LGPL
 * Arduino IPAddress.cpp.
 *
 * Copyright (c) 2026 Nicholas Newdigate
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <Arduino.h>
#include "IPAddress.h"

// Parse a strict dotted-quad IPv4 address ("a.b.c.d", each field a decimal
// number 0-255).  Returns true and stores the address on success.  Returns
// false on any malformed input: wrong number of fields, empty fields, a
// field greater than 255, or any character that is not a digit or a
// separating dot.  On failure the stored address is left unchanged.
bool IPAddress::fromString(const char *address)
{
	if (address == nullptr) return false;

	uint8_t parsed[4];
	const char *s = address;

	for (int field = 0; field < 4; field++) {
		if (field > 0) {
			if (*s != '.') return false;	// fields separated by single dots
			s++;
		}
		if (*s < '0' || *s > '9') return false;	// each field needs >= 1 digit
		unsigned int value = 0;
		while (*s >= '0' && *s <= '9') {
			value = value * 10 + (unsigned int)(*s - '0');
			if (value > 255) return false;	// octet out of range
			s++;
		}
		parsed[field] = (uint8_t)value;
	}
	if (*s != '\0') return false;		// trailing garbage (incl. 5th field)

	_address.bytes[0] = parsed[0];
	_address.bytes[1] = parsed[1];
	_address.bytes[2] = parsed[2];
	_address.bytes[3] = parsed[3];
	return true;
}

// Print the address in dotted-quad form ("a.b.c.d") to a Print sink and
// return the number of characters written.
size_t IPAddress::printTo(Print &p) const
{
	size_t n = 0;
	n += p.print(_address.bytes[0]);
	n += p.print('.');
	n += p.print(_address.bytes[1]);
	n += p.print('.');
	n += p.print(_address.bytes[2]);
	n += p.print('.');
	n += p.print(_address.bytes[3]);
	return n;
}
