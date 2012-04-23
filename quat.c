/*
 * Copyright (c) 2010, Regents of the University of California
 * All rights reserved->
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer->
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution->
 * - Neither the name of the University of California, Berkeley nor the names
 *   of its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission->
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED-> IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE->
 *
 *
 * Quaternion Library
 *
 * by Humphrey Hu
 *
 * v-> beta
 *
 * Revisions:
 *  Humphrey      2011-10-07    Initial release
 *                      
 * 
 */

 #include "quat.h"
 #include <stdlib.h>
 #include <math.h>
 
 void quatRotate(Quaternion *q, Quaternion *v, Quaternion *result) {
 
    Quaternion temp;
	quatConj(q, &temp);
	quatMult(q, v, result);
	quatMult(result, &temp, result);
 
 }
 
 void quatConj(Quaternion *q, Quaternion *result) {
 
	result->w = q->w;
	result->x = -q->x;
	result->y = -q->y;
	result->z = -q->z;
 
 }
 
 void quatMult(Quaternion *a, Quaternion *b, Quaternion *c) {
 
	float w, x, y, z;
	w = a->w*b->w - a->x*b->x - a->y*b->y - a->z*b->z;
	x = a->w*b->x + b->w*a->x + a->y*b->z - a->z*b->y;
	y = a->w*b->y + b->w*a->y - a->x*b->z + a->z*b->x;
	z = a->w*b->z + b->w*a->z + a->x*b->y - a->y*b->x;
	
	c->w = w;
	c->x = x;
	c->y = y;
	c->z = z;
 
 }
 
 void quatNormalize(Quaternion *a) {
 
	float norm = sqrtf(a->w*a->w + a->x*a->x + a->y*a->y + a->z*a->z);
	a->w = a->w/norm;
	a->x = a->x/norm;
	a->y = a->y/norm;
	a->z = a->z/norm;
	
 }
