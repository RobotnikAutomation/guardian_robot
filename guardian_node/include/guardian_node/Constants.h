/** \file Constants.h
 * \version 080411
 * \date    2011
 *
 * \brief class for RoboteqDevice servo driver
 * (C) Roboteq Inc., 2012
*/

#ifndef __Constants__
#define __Constants__

//SetCommand Long
#define _GO 0

#define _MOTCMD 1
#define _MOTPOS 2
#define _MOTVEL 3
#define _SENCNTR 4
#define _SBLCNTR 5
#define _VAR 6

#define _ACCEL 7
#define _DECEL 8

#define _DOUT 9
#define _DSET 10
#define _DRES 11


#define _HOME 13
#define _ESTOP 14
#define _MGO 15

//SetCommandShort
#define _G 0

#define _M 1
#define _P 2
#define _S 3
#define _C 4
#define _CB 5
#define _VAR 6

#define _AC 7
#define _DC 8

#define _DS 9
#define _D1 10
#define _D0 11


#define _H 13
#define _EX 14
#define _MG 15

//GetValue Long
#define _MOTAMPS 0

#define _MOTCMD 1
#define _MOTPWR 2
#define _ABSPEED 3
#define _ABCNTR 4
#define _BLCNTR 5
#define _VAR 6

#define _RELSPEED 7
#define _RELCNTR 8
#define _BLRCNTR 9
#define _BLSPEED 10
#define _BLRSPEED 11
#define _BATAMPS 12
#define _VOLTS 13
#define _DIGIN 14
#define _DIN 15
#define _ANAIN 16
#define _PLSIN 17
#define _TEMP 18
#define _FEEDBK 19
#define _STFLAG 20
#define _FLTFLAG 21

#define _DIGOUT 23
#define _LPERR 24

#define _CMDSER 25
#define _CMDANA 26
#define _CMDPLS 27

#define _TIME 28

#define _LOCKED 29

//GetValue Short
#define _A 0

#define _M 1
#define _P 2
#define _S 3
#define _C 4
#define _CB 5
#define _VAR 6

#define _SR 7
#define _CR 8
#define _CBR 9
#define _BS 10
#define _BSR 11
#define _BA 12
#define _V 13
#define _D 14
#define _DI 15
#define _AI 16
#define _PI 17
#define _T 18
#define _F 19
#define _FS 20
#define _FF 21

#define _DO 23
#define _E 24

#define _CIS 25
#define _CIA 26
#define _CIP 27

#define _TM 28

#define _LK 29

//Configuration
#define _CAD 1
#define _OVL 2
#define _UVL 3
#define _THLD 4
#define _MXMD 5
#define _PWMF 6

#define _CPRI 7
#define _RWD 8
#define _ECHOF 9
#define _PMS 10
#define _ACS 11
#define _AMS 12
#define _CLIN 13
#define _DFC 14

#define _DINA 15
#define _DINL 16
#define _DOA 17
#define _DOL 18

#define _AMOD 19
#define _AMIN 20
#define _AMAX 21
#define _ACTR 22
#define _ADB 23
#define _ALIN 24
#define _AINA 25
#define _AMINA 26
#define _AMAXA 27
#define _APOL 28

#define _PMOD 29
#define _PMIN 30
#define _PMAX 31
#define _PCTR 32
#define _PDB 33
#define _PLIN 34
#define _PINA 35
#define _PMINA 36
#define _PMAXA 37
#define _PPOL 38

#define _MMOD 39
#define _MXPF 40
#define _MXPR 41
#define _ALIM 42
#define _ATRIG 43
#define _ATGA 44
#define _ATGD 45
#define _KP 46
#define _KI 47
#define _KD 48
#define _PIDM 49
#define _ICAP 50
#define _MAC 51
#define _MDEC 52
#define _MVEL 53
#define _MXRPM 54
#define _MXTRN 54
#define _CLERD 55

#define _BPOL 56
#define _BLSTD 57
#define _BLFB 58
#define _BHOME 59
#define _BLL 60
#define _BHL 61
#define _BLLA 62
#define _BHLA 63

#define _SXC 64
#define _SXM 65









#define _EMOD 72
#define _EPPR 73
#define _ELL 74
#define _EHL 75
#define _ELLA 76
#define _EHLA 77
#define _KPC1 78
#define _KPC2 79
#define _KIC1 80
#define _KIC2 81
#define _KDC1 82
#define _KDC2 83
#define _EHOME 84

#endif
