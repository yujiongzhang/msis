#pragma once
#ifndef DEFINE_STS1000_H
#define DEFINE_STS1000_H



#define RECVBUFSIZE 200000    //实时数据流接收缓冲区大小
#define DATAARYSIZE 200000  //数据大数组大小
#define ANSPKTSIZE 516 //一个应答包大小
#define HMSPPIDATASIZE 500 //sts1000 PPI模式数据量500

#define SCANRANGE 10.0 //扫描范围



//**************************应答包帧头***************************
typedef struct // Sonar Echo Data
{
    unsigned char ucHeader;// 0xFE
    unsigned char ucID; // 0x1 to 0xfD
    unsigned char ucStatus; // Bit1:0 - 00 8bits数据，一字节代表一数据点
    unsigned char ucTemp; // 温度，100+T，T∈(-99,152)，单位0.5℃
    unsigned char ucDataLo; // Echo Package length =
    unsigned char ucDataHi; // (((ucDataHi&0x7f)<<7)|(ucDataLo&0x7f))
    unsigned char ucAngleLo; // nAngle =
    unsigned char ucAngleHi; // (((ucAngleHi&0x7f)<<7)|(ucAngleLo&0x7f))
    unsigned char ucCPSLo; // 罗经=
    unsigned char ucCPSHi; // (((ucCPSHi&0x7f)<<7)|(ucCPSLo&0x7f))
    unsigned char ucPRSLo; // 压力=
    unsigned char ucPRSHi; // (((ucPRSHi&0x7f)<<7)|(ucPRSLo&0x7f))
    unsigned char ucPitchLo; // 纵倾=
    unsigned char ucPitchHi; // (((ucPitchHi&0x7f)<<7)|(ucPitchLo&0x7f))
    unsigned char ucRollLo; // 横摇=
    unsigned char ucRollHi; // (((ucRollHi&0x7f)<<7)|(ucRollLo&0x7f))
}ANSHEAD_STS1000;


#endif // DEFINE_STS1000_H
