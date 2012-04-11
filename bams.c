/*
 * Copyright (c) 2009 - 2010, Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the University of California, Berkeley nor the names
 *   of its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * Binary Angle Measurement System (BAMS) Implementation
 *
 * by Humphrey Hu
 *
 * v.alpha
 *
 * Revisions:
 *  Humphrey Hu			 2011-10-23	   Initial implementation
 * 
 */

#include "bams.h"
#include "math.h"

// Conversion constants
#define BAMS16_TO_RAD		(9.58737992e-5)
#define BAMS16_TO_DEG		(5.49316406e-3)
#define BAMS32_TO_RAD		(1.46291808e-9)
#define BAMS32_TO_DEG		(8.38190317e-8)

#define RAD_TO_BAMS16		(10430.3784)
#define DEG_TO_BAMS16		(182.044444)
#define DEG_TO_BAMS32		(11930464.7)
#define RAD_TO_BAMS32		(683565276.0)

#define BAMS16_LSB2RAD		(9.58737992e-5) 		
#define BAMS32_LSB2RAD		(1.46291808e-9)

// Other constants
#define _2PI				(6.28318531)
#define PI					(3.14159265)
#define PI_4				(0.785398163

float bams16ToFloatRad(bams16_t b) {
    return b*BAMS16_TO_RAD;
}
float bams16ToFloatDeg(bams16_t b) {
    return b*BAMS16_TO_DEG;
}
bams16_t floatToBams16Rad(float f) {
    return (bams16_t) (((f > PI) ? (f - _2PI) : f)*RAD_TO_BAMS16);
}
bams16_t floatToBams16Deg(float f) {
    return (bams16_t) (((f > 180.0) ? (f - 360.0) : f)*DEG_TO_BAMS16);
}

float bams32ToFloatRad(bams32_t b) {
    return b*BAMS32_TO_RAD;
}
float bams32ToFloatDeg(bams32_t b) {
    return b*BAMS32_TO_DEG;
}
bams32_t floatToBams32Rad(float f) {
    return (bams32_t) (((f > PI) ? (f - _2PI) : f)*RAD_TO_BAMS32);
}
bams32_t floatToBams32Deg(float f) {
    return (bams32_t) (((f > 180.0) ? (f - 360.0) : f)*DEG_TO_BAMS32);
}
 
bams16_t bams32ToBams16(bams32_t b) {            
    return (bams16_t) (b & 0x0080) ? (b >> 16) + 1 : b >> 16;
}
bams32_t bams16ToBams32(bams16_t b) {
    return (bams32_t) b << 16;
}
 
// Need 2^i + 1 values to cover 0 and pi/4, where i is # index bits
// We shift over 6 bits to get 10 significant bits, which we then use symmetry to
// further segment by 4, bringing us to 8 index bits (2^8 + 1 = 257)
#define SIN_TABLE_SIZE              (257)
#define SIN_SIGBITS_SHIFT           (6)         // Right shift amount to get significant bits
#define SIN_SIGBITS16_UPPER_MASK    (0xFFC0)    // Top 10 bits
#define SIN_SIGBITS16_LOWER_MASK    (0x003F)    // Bottom 6 bits
#define SIN_SIGBITS32_UPPER_MASK    (0xFFC00000)// Top 10 bits
#define SIN_SIGBITS32_LOWER_MASK    (0x003FFFFF)// Bottom 22 bits
#define SIN_FINE_STEP16             (0.015625)
#define SIN_FINE_STEP32             (2.38418579E-7)
const float sin_table[] = {
    0.0, 0.00613588464915, 0.0122715382857, 0.0184067299058, 0.0245412285229, 0.0306748031766, 0.0368072229414, 0.0429382569349, 0.0490676743274, 0.0551952443497, 
    0.0613207363022, 0.0674439195637, 0.0735645635997, 0.0796824379714, 0.0857973123444, 0.0919089564971, 0.0980171403296, 0.104121633872, 0.110222207294, 0.116318630912, 
    0.122410675199, 0.128498110794, 0.134580708507, 0.140658239333, 0.146730474455, 0.152797185258, 0.158858143334, 0.16491312049, 0.17096188876, 0.177004220412, 
    0.183039887955, 0.18906866415, 0.195090322016, 0.201104634842, 0.207111376192, 0.213110319916, 0.219101240157, 0.22508391136, 0.231058108281, 0.237023605994, 
    0.242980179903, 0.248927605746, 0.254865659605, 0.260794117915, 0.266712757475, 0.27262135545, 0.278519689385, 0.284407537211, 0.290284677254, 0.296150888244, 
    0.302005949319, 0.307849640042, 0.313681740399, 0.319502030816, 0.325310292162, 0.33110630576, 0.336889853392, 0.342660717312, 0.348418680249, 0.35416352542, 
    0.359895036535, 0.365612997805, 0.371317193952, 0.377007410216, 0.382683432365, 0.388345046699, 0.393992040061, 0.399624199846, 0.405241314005, 0.410843171058, 
    0.416429560098, 0.4220002708, 0.42755509343, 0.433093818853, 0.438616238539, 0.44412214457, 0.449611329655, 0.455083587126, 0.460538710958, 0.465976495768, 
    0.471396736826, 0.476799230063, 0.482183772079, 0.487550160148, 0.49289819223, 0.498227666973, 0.503538383726, 0.508830142543, 0.514102744193, 0.519355990166, 
    0.524589682678, 0.529803624686, 0.534997619887, 0.54017147273, 0.545324988422, 0.550457972937, 0.55557023302, 0.560661576197, 0.565731810784, 0.570780745887, 
    0.575808191418, 0.580813958096, 0.585797857456, 0.590759701859, 0.595699304492, 0.600616479384, 0.605511041404, 0.610382806276, 0.615231590581, 0.620057211763, 
    0.624859488142, 0.629638238915, 0.634393284164, 0.639124444864, 0.64383154289, 0.648514401022, 0.653172842954, 0.657806693297, 0.66241577759, 0.666999922304, 
    0.671558954847, 0.676092703575, 0.680600997795, 0.685083667773, 0.689540544737, 0.69397146089, 0.698376249409, 0.702754744457, 0.707106781187, 0.711432195745, 
    0.715730825284, 0.720002507961, 0.724247082951, 0.728464390448, 0.732654271672, 0.736816568877, 0.740951125355, 0.745057785441, 0.749136394523, 0.753186799044, 
    0.757208846506, 0.761202385484, 0.765167265622, 0.769103337646, 0.773010453363, 0.776888465673, 0.780737228572, 0.784556597156, 0.788346427627, 0.7921065773, 
    0.795836904609, 0.799537269108, 0.803207531481, 0.806847553544, 0.810457198253, 0.814036329706, 0.817584813152, 0.821102514991, 0.824589302785, 0.828045045258, 
    0.831469612303, 0.834862874986, 0.838224705555, 0.841554977437, 0.84485356525, 0.848120344803, 0.851355193105, 0.854557988365, 0.85772861, 0.860866938638, 
    0.863972856122, 0.867046245516, 0.870086991109, 0.873094978418, 0.876070094195, 0.879012226429, 0.881921264348, 0.884797098431, 0.887639620403, 0.890448723245, 
    0.893224301196, 0.895966249756, 0.898674465694, 0.901348847046, 0.903989293123, 0.906595704515, 0.909167983091, 0.911706032005, 0.914209755704, 0.916679059921, 
    0.91911385169, 0.921514039342, 0.923879532511, 0.926210242138, 0.928506080473, 0.930766961079, 0.932992798835, 0.935183509939, 0.937339011913, 0.939459223602, 
    0.941544065183, 0.943593458162, 0.945607325381, 0.947585591018, 0.949528180593, 0.951435020969, 0.953306040354, 0.955141168306, 0.956940335732, 0.958703474896, 
    0.960430519416, 0.962121404269, 0.963776065795, 0.965394441698, 0.966976471045, 0.968522094274, 0.970031253195, 0.971503890986, 0.972939952206, 0.974339382786, 
    0.975702130039, 0.977028142658, 0.97831737072, 0.979569765685, 0.980785280403, 0.98196386911, 0.983105487431, 0.984210092387, 0.985277642389, 0.986308097245, 
    0.987301418158, 0.988257567731, 0.989176509965, 0.990058210262, 0.990902635428, 0.991709753669, 0.992479534599, 0.993211949235, 0.993906970002, 0.994564570734, 
    0.995184726672, 0.995767414468, 0.996312612183, 0.996820299291, 0.997290456679, 0.997723066644, 0.9981181129, 0.998475580573, 0.998795456205, 0.999077727753, 
    0.999322384588, 0.999529417501, 0.999698818696, 0.999830581796, 0.999924701839, 0.999981175283, 1.0, 0.999981175283
};
 
// Quick lookup implementation
// Runs full range at ~43 cycles
float bams16Sin(bams16_t b) {
    
    if(b < 0) {
        if(b < -BAMS16_PI_2) {
            b = b - BAMS16_PI;
        } else {
            b = -b;
        }
        return -sin_table[(unsigned int) b >> SIN_SIGBITS_SHIFT];
    } else {
        if(b > BAMS16_PI_2) {
            b = BAMS16_PI - b;
        } else {
            // b = b;
        }
        return sin_table[(unsigned int) b >> SIN_SIGBITS_SHIFT];
    }
    
}

float bams16Cos(bams16_t b) {
    // cos(x) = sin(x + pi/2)        
    return bams16Sin(b + BAMS16_PI_2);
    
}

// High precision implementation
// Runs full range at ~370 cycles
float bams16SinFine(bams16_t b) {

    float v0, v1, r;
    
    v0 = bams16Sin(b & SIN_SIGBITS16_UPPER_MASK);
    v1 = bams16Sin((b & SIN_SIGBITS16_UPPER_MASK) + SIN_SIGBITS16_LOWER_MASK + 1);
    r = (b & SIN_SIGBITS16_LOWER_MASK)*SIN_FINE_STEP16;    
    return v0 + (v1 - v0)*r;
    
}

// High precision implementation
// ~370 cycles
float bams16CosFine(bams16_t b) {
 
   return bams16SinFine(b + BAMS16_PI_2);
 
}

float bams16Tan(bams16_t b) {
    return bams16Sin(b)/bams16Cos(b);
}

float bams16TanFine(bams16_t b) {
    return bams16SinFine(b)/bams16CosFine(b);
}

float bams32Sin(bams32_t b) {

    return bams16Sin(bams32ToBams16(b));

}

float bams32Cos(bams32_t b) {

    return bams16Cos(bams32ToBams16(b));

}

float bams32SinFine(bams32_t b) {

    float v0, v1, r;
    
    v0 = bams32Sin(b & SIN_SIGBITS32_UPPER_MASK);
    v1 = bams32Sin((b & SIN_SIGBITS32_UPPER_MASK) + SIN_SIGBITS32_LOWER_MASK + 1);
    r = (b & SIN_SIGBITS32_LOWER_MASK)*SIN_FINE_STEP32;    
    return v0 + (v1 - v0)*r;    

}

float bams32CosFine(bams32_t b) {

    return bams32SinFine(b + BAMS32_PI_2);

}

float bams32Tan(bams32_t b) {

    return bams32Sin(b)/bams32Cos(b);

}

float bams32TanFine(bams32_t b) {
    
    return bams32SinFine(b)/bams32CosFine(b);
    
}

#define ASIN_TABLE_NUM		(256) // +1 extra value for linear interpolation
#define ASIN_STEP			(255.0)
const bams16_t bams_asin_table[] = {
    0x0, 0x28, 0x51, 0x7a, 0xa3, 0xcc, 0xf5, 0x11e, 0x147, 0x170, 
    0x199, 0x1c2, 0x1eb, 0x213, 0x23c, 0x265, 0x28e, 0x2b7, 0x2e0, 0x309, 
    0x332, 0x35b, 0x384, 0x3ae, 0x3d7, 0x400, 0x429, 0x452, 0x47b, 0x4a4, 
    0x4cd, 0x4f7, 0x520, 0x549, 0x572, 0x59c, 0x5c5, 0x5ee, 0x618, 0x641, 
    0x66a, 0x694, 0x6bd, 0x6e7, 0x710, 0x73a, 0x763, 0x78d, 0x7b7, 0x7e0, 
    0x80a, 0x834, 0x85e, 0x887, 0x8b1, 0x8db, 0x905, 0x92f, 0x959, 0x983, 
    0x9ad, 0x9d7, 0xa01, 0xa2b, 0xa56, 0xa80, 0xaaa, 0xad5, 0xaff, 0xb29, 
    0xb54, 0xb7f, 0xba9, 0xbd4, 0xbff, 0xc29, 0xc54, 0xc7f, 0xcaa, 0xcd5, 
    0xd00, 0xd2b, 0xd56, 0xd81, 0xdad, 0xdd8, 0xe04, 0xe2f, 0xe5b, 0xe86, 
    0xeb2, 0xede, 0xf09, 0xf35, 0xf61, 0xf8d, 0xfb9, 0xfe6, 0x1012, 0x103e, 
    0x106b, 0x1097, 0x10c4, 0x10f0, 0x111d, 0x114a, 0x1177, 0x11a4, 0x11d1, 0x11fe, 
    0x122c, 0x1259, 0x1286, 0x12b4, 0x12e2, 0x130f, 0x133d, 0x136b, 0x1399, 0x13c8, 
    0x13f6, 0x1424, 0x1453, 0x1482, 0x14b0, 0x14df, 0x150e, 0x153d, 0x156c, 0x159c, 
    0x15cb, 0x15fb, 0x162b, 0x165b, 0x168b, 0x16bb, 0x16eb, 0x171b, 0x174c, 0x177d, 
    0x17ae, 0x17df, 0x1810, 0x1841, 0x1873, 0x18a4, 0x18d6, 0x1908, 0x193a, 0x196c, 
    0x199f, 0x19d2, 0x1a04, 0x1a37, 0x1a6b, 0x1a9e, 0x1ad2, 0x1b06, 0x1b3a, 0x1b6e, 
    0x1ba2, 0x1bd7, 0x1c0c, 0x1c41, 0x1c76, 0x1cac, 0x1ce1, 0x1d17, 0x1d4e, 0x1d84, 
    0x1dbb, 0x1df2, 0x1e29, 0x1e61, 0x1e98, 0x1ed1, 0x1f09, 0x1f42, 0x1f7b, 0x1fb4, 
    0x1fed, 0x2027, 0x2062, 0x209c, 0x20d7, 0x2112, 0x214e, 0x218a, 0x21c6, 0x2203, 
    0x2240, 0x227e, 0x22bc, 0x22fa, 0x2339, 0x2378, 0x23b8, 0x23f8, 0x2439, 0x247a, 
    0x24bb, 0x24fe, 0x2540, 0x2584, 0x25c8, 0x260c, 0x2651, 0x2697, 0x26dd, 0x2724, 
    0x276c, 0x27b4, 0x27fe, 0x2848, 0x2892, 0x28de, 0x292b, 0x2978, 0x29c6, 0x2a16, 
    0x2a66, 0x2ab8, 0x2b0a, 0x2b5e, 0x2bb3, 0x2c09, 0x2c61, 0x2cba, 0x2d14, 0x2d70, 
    0x2dce, 0x2e2e, 0x2e8f, 0x2ef3, 0x2f59, 0x2fc1, 0x302c, 0x3099, 0x3109, 0x317d, 
    0x31f4, 0x326f, 0x32ef, 0x3373, 0x33fd, 0x348d, 0x3524, 0x35c4, 0x366e, 0x3724, 
    0x37eb, 0x38c6, 0x39be, 0x3ae4, 0x3c63, 0x4000, 0x3c63 // Extra value for linear interp bounds

};

// Standard precision implementation
// 330 cycles
bams16_t bams16Asin(float f) {
    
    if(f >= 0.0) {
        return bams_asin_table[(unsigned int) (f*ASIN_STEP)];
    } else {
        return -bams_asin_table[(unsigned int) (-f*ASIN_STEP)];
    }
}

bams16_t bams16Acos(float f) {

    if(f >= 0.0) {
        return -bams_asin_table[(unsigned int) (f*ASIN_STEP)] + BAMS16_PI_2;
    } else {
        return bams_asin_table[(unsigned int) (-f*ASIN_STEP)] + BAMS16_PI_2;
    }

}

// High precision implementation
// 1100 cycles
bams16_t bams16AsinFine(float f) {

    float fract, pint;
    unsigned int index;
    bams16_t b1, b2, db;
    
    if(f >= 0.0) {
        fract = modff(f*ASIN_STEP, &pint);
        index = (unsigned int) pint;
        b1 = bams_asin_table[index];
        b2 = bams_asin_table[index + 1];
        db = (unsigned int) (fract*(b2 - b1));
        return b1 + db;
    } else {
        fract = modff(-f*ASIN_STEP, &pint);
        index = (unsigned int) pint;
        b1 = bams_asin_table[index];
        b2 = bams_asin_table[index + 1];
        db = (unsigned int) (fract*(b2 - b1));
        return -(b1 + db);
    }
    
}

bams16_t bams16AcosFine(float f) {

    float fract, pint;
    unsigned int index;
    bams16_t b1, b2, db;
    
    if(f >= 0.0) {
        fract = modff(f*ASIN_STEP, &pint);
        index = (unsigned int) pint;
        b1 = bams_asin_table[index];
        b2 = bams_asin_table[index + 1];
        db = (unsigned int) (fract*(b2 - b1));
        return -(b1 + db - BAMS16_PI_2);
    } else {
        fract = modff(-f*ASIN_STEP, &pint);
        index = (unsigned int) pint;
        b1 = bams_asin_table[index];
        b2 = bams_asin_table[index + 1];
        db = (unsigned int) (fract*(b2 - b1));
        return b1 + db + BAMS16_PI_2;
    }
    
}

#define ATAN_TABLE_NUM 	(256)
#define ATAN_STEP		(255.0)

const bams16_t bams_atan_table[] = {
    0x0, 0x28, 0x51, 0x7a, 0xa3, 0xcc, 0xf5, 0x11e, 0x147, 0x16f, 
    0x198, 0x1c1, 0x1ea, 0x213, 0x23c, 0x264, 0x28d, 0x2b6, 0x2df, 0x307, 
    0x330, 0x359, 0x381, 0x3aa, 0x3d2, 0x3fb, 0x423, 0x44c, 0x474, 0x49d, 
    0x4c5, 0x4ed, 0x516, 0x53e, 0x566, 0x58e, 0x5b6, 0x5de, 0x606, 0x62e, 
    0x656, 0x67e, 0x6a6, 0x6ce, 0x6f6, 0x71d, 0x745, 0x76d, 0x794, 0x7bc, 
    0x7e3, 0x80a, 0x832, 0x859, 0x880, 0x8a7, 0x8ce, 0x8f5, 0x91c, 0x943, 
    0x96a, 0x991, 0x9b7, 0x9de, 0xa04, 0xa2b, 0xa51, 0xa77, 0xa9e, 0xac4, 
    0xaea, 0xb10, 0xb36, 0xb5c, 0xb81, 0xba7, 0xbcd, 0xbf2, 0xc18, 0xc3d, 
    0xc62, 0xc88, 0xcad, 0xcd2, 0xcf7, 0xd1b, 0xd40, 0xd65, 0xd8a, 0xdae, 
    0xdd2, 0xdf7, 0xe1b, 0xe3f, 0xe63, 0xe87, 0xeab, 0xecf, 0xef3, 0xf16, 
    0xf3a, 0xf5d, 0xf80, 0xfa4, 0xfc7, 0xfea, 0x100d, 0x102f, 0x1052, 0x1075, 
    0x1097, 0x10ba, 0x10dc, 0x10fe, 0x1120, 0x1143, 0x1164, 0x1186, 0x11a8, 0x11ca, 
    0x11eb, 0x120d, 0x122e, 0x124f, 0x1270, 0x1291, 0x12b2, 0x12d3, 0x12f4, 0x1314, 
    0x1335, 0x1355, 0x1376, 0x1396, 0x13b6, 0x13d6, 0x13f6, 0x1416, 0x1435, 0x1455, 
    0x1474, 0x1494, 0x14b3, 0x14d2, 0x14f1, 0x1510, 0x152f, 0x154e, 0x156d, 0x158b, 
    0x15aa, 0x15c8, 0x15e6, 0x1604, 0x1622, 0x1640, 0x165e, 0x167c, 0x1699, 0x16b7, 
    0x16d4, 0x16f2, 0x170f, 0x172c, 0x1749, 0x1766, 0x1782, 0x179f, 0x17bc, 0x17d8, 
    0x17f5, 0x1811, 0x182d, 0x1849, 0x1865, 0x1881, 0x189d, 0x18b8, 0x18d4, 0x18ef, 
    0x190b, 0x1926, 0x1941, 0x195c, 0x1977, 0x1992, 0x19ad, 0x19c7, 0x19e2, 0x19fc, 
    0x1a17, 0x1a31, 0x1a4b, 0x1a65, 0x1a7f, 0x1a99, 0x1ab3, 0x1acc, 0x1ae6, 0x1aff, 
    0x1b19, 0x1b32, 0x1b4b, 0x1b64, 0x1b7d, 0x1b96, 0x1baf, 0x1bc8, 0x1be0, 0x1bf9, 
    0x1c11, 0x1c2a, 0x1c42, 0x1c5a, 0x1c72, 0x1c8a, 0x1ca2, 0x1cba, 0x1cd1, 0x1ce9, 
    0x1d00, 0x1d18, 0x1d2f, 0x1d46, 0x1d5d, 0x1d74, 0x1d8b, 0x1da2, 0x1db9, 0x1dd0, 
    0x1de6, 0x1dfd, 0x1e13, 0x1e2a, 0x1e40, 0x1e56, 0x1e6c, 0x1e82, 0x1e98, 0x1eae, 
    0x1ec4, 0x1ed9, 0x1eef, 0x1f04, 0x1f1a, 0x1f2f, 0x1f44, 0x1f59, 0x1f6e, 0x1f83, 
    0x1f98, 0x1fad, 0x1fc2, 0x1fd6, 0x1feb, 0x2000
};

// Standard implementation
// 1000 cycles
bams16_t bams16Atan2(float y, float x) {
    
    float div;
    unsigned char yneg, xneg, invert, temp;
    bams16_t b;
    
    yneg = y < 0.0;
    xneg = x < 0.0;
    
    y = (yneg) ? -y : y;
    x = (xneg) ? -x : x;
    
    // Add pi/2 to tan argument and flip the tan
    // This lets us use a quarter table
    if(y > x) {
        div = x/y;
        temp = yneg;
        yneg = xneg;
        xneg = temp;
        invert = 1;
    } else {
        div = y/x;
        invert = 0;
    }
    
    b = bams_atan_table[(unsigned int) (div*ATAN_STEP)];
    b = (xneg) ? BAMS16_PI - b : b;
    b = (yneg) ? -b : b;
    return (invert) ? -b + BAMS16_PI_2 : b;
    
}
