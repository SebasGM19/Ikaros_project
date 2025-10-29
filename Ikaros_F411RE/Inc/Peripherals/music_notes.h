/*
 * music_notes.h
 *
 *  Created on: Aug 2, 2025
 *      Author: sebas
 */

#ifndef MUSIC_NOTES_H_
#define MUSIC_NOTES_H_

typedef enum{

	/* Octava 0 mas grave  */
	Do_0 		= 16,
	Do_sh_0 	= 17,
	Re_0 		= 18,
	Re_sh_0 	= 19,
	Mi_0 		= 20,
	Fa_0		= 22,
	Fa_sh_0 	= 23,
	Sol_0 		= 24,
	Sol_sh_0 	= 26,
	La_0 		= 27,
	La_sh_0 	= 29,
	Si_0 		= 31,

	/* Octava 1*/
	Do_1 		= 33,
	Do_sh_1 	= 34,
	Re_1 		= 37,
	Re_sh_1 	= 38,
	Mi_1 		= 41,
	Fa_1 		= 43,
	Fa_sh_1 	= 46,
	Sol_1 		= 49,
	Sol_sh_1 	= 52,
	La_1 		= 55,
	La_sh_1 	= 58,
	Si_1 		= 61,

	/* Octava 2*/
	Do_2 		= 65,
	Do_sh_2 	= 69,
	Re_2 		= 73,
	Re_sh_2 	= 78,
	Mi_2 		= 82,
	Fa_2 		= 87,
	Fa_sh_2 	= 92,
	Sol_2 		= 98,
	Sol_sh_2 	= 104,
	La_2 		= 110,
	La_sh_2 	= 116,
	Si_2 		= 123,

	/* Octava 3*/
	Do_3 		= 131,
	Do_sh_3 	= 138,
	Re_3 		= 147,
	Re_sh_3 	= 155,
	Mi_3 		= 165,
	Fa_3 		= 174,
	Fa_sh_3 	= 185,
	Sol_3 		= 196,
	Sol_sh_3 	= 207,
	La_3 		= 220,
	La_sh_3 	= 233,
	Si_3 		= 247,

	/* Octava 4 la mas usada*/
	Do_4 		= 261,
	Do_sh_4 	= 277,
	Re_4 		= 293,
	Re_sh_4 	= 311,
	Mi_4 		= 329,
	Fa_4 		= 349,
	Fa_sh_4 	= 370,
	Sol_4 		= 392,
	Sol_sh_4 	= 415,
	La_4 		= 440,
	La_sh_4 	= 466,
	Si_4 		= 494,

	/* Octava 5*/
	Do_5 		= 523,
	Do_sh_5 	= 554,
	Re_5 		= 587,
	Re_sh_5 	= 622,
	Mi_5 		= 659,
	Fa_5 		= 698,
	Fa_sh_5 	= 740,
	Sol_5 		= 784,
	Sol_sh_5 	= 830,
	La_5 		= 880,
	La_sh_5 	= 932,
	Si_5 		= 988,


	/* Octava 6*/
	Do_6 		= 1046,
	Do_sh_6 	= 1109,
	Re_6 		= 1174,
	Re_sh_6 	= 1244,
	Mi_6 		= 1318,
	Fa_6 		= 1397,
	Fa_sh_6 	= 1480,
	Sol_6 		= 1568,
	Sol_sh_6 	= 1661,
	La_6 		= 1760,
	La_sh_6 	= 1864,
	Si_6 		= 1975,



	/* Octava 7*/
	Do_7 		= 2093,
	Do_sh_7 	= 2217,
	Re_7 		= 2349,
	Re_sh_7 	= 2489,
	Mi_7 		= 2637,
	Fa_7 		= 2794,
	Fa_sh_7 	= 2960,
	Sol_7 		= 3136,
	Sol_sh_7 	= 3322,
	La_7 		= 3520,
	La_sh_7 	= 3729,
	Si_7 		= 3951,


	/* Octava 8*/
	Do_8 		= 4186,
	Do_sh_8 	= 4435,
	Re_8 		= 4699,
	Re_sh_8 	= 4978,
	Mi_8 		= 5274,
	Fa_8 		= 5587,
	Fa_sh_8 	= 5920,
	Sol_8 		= 6272,
	Sol_sh_8 	= 6645,
	La_8 		= 7040,
	La_sh_8 	= 7458,
	Si_8 		= 7902,

}Frecuency_notes_spanish_t;




typedef enum{
	none = 100,
	/* Octava 0 mas grave  */
	C_0 	= 16,
	C_sh_0 	= 17,
	D_0 	= 18,
	D_sh_0 	= 19,
	E_0 	= 20,
	F_0		= 22,
	F_sh_0 	= 23,
	G_0 	= 24,
	G_sh_0 	= 26,
	A_0 	= 27,
	A_sh_0 	= 29,
	B_0 	= 31,

	/* Octava 1*/
	C_1 	= 33,
	C_sh_1 	= 34,
	D_1 	= 37,
	D_sh_1 	= 38,
	E_1 	= 41,
	F_1 	= 43,
	F_sh_1 	= 46,
	G_1 	= 49,
	G_sh_1 	= 52,
	A_1 	= 55,
	A_sh_1 	= 58,
	B_1 	= 61,

	/* Octava 2*/
	C_2 	= 65,
	C_sh_2 	= 69,
	D_2 	= 73,
	D_sh_2 	= 78,
	E_2 	= 82,
	F_2 	= 87,
	F_sh_2 	= 92,
	G_2 	= 98,
	G_sh_2 	= 104,
	A_2 	= 110,
	A_sh_2 	= 116,
	B_2 	= 123,

	/* Octava 3*/
	C_3 	= 131,
	C_sh_3 	= 138,
	D_3 	= 147,
	D_sh_3 	= 155,
	E_3 	= 165,
	F_3 	= 174,
	F_sh_3 	= 185,
	G_3 	= 196,
	G_sh_3 	= 207,
	A_3 	= 220,
	A_sh_3 	= 233,
	B_3 	= 247,

	/* Octava 4 la mas usada*/
	C_4 	= 261,
	C_sh_4 	= 277,
	D_4 	= 293,
	D_sh_4 	= 311,
	E_4 	= 329,
	F_4 	= 349,
	F_sh_4 	= 370,
	G_4 	= 392,
	G_sh_4 	= 415,
	A_4 	= 440,
	A_sh_4 	= 466,
	B_4 	= 494,

	/* Octava 5*/
	C_5 	= 523,
	C_sh_5 	= 554,
	D_5 	= 587,
	D_sh_5 	= 622,
	E_5 	= 659,
	F_5 	= 698,
	F_sh_5 	= 740,
	G_5 	= 784,
	G_sh_5 	= 830,
	A_5 	= 880,
	A_sh_5 	= 932,
	B_5 	= 988,


	/* Octava 6*/
	C_6 	= 1046,
	C_sh_6 	= 1109,
	D_6 	= 1174,
	D_sh_6 	= 1244,
	E_6 	= 1318,
	F_6 	= 1397,
	F_sh_6 	= 1480,
	G_6 	= 1568,
	G_sh_6 	= 1661,
	A_6 	= 1760,
	A_sh_6 	= 1864,
	B_6 	= 1975,



	/* Octava 7*/
	C_7 	= 2093,
	C_sh_7 	= 2217,
	D_7 	= 2349,
	D_sh_7 	= 2489,
	E_7 	= 2637,
	F_7 	= 2794,
	F_sh_7 	= 2960,
	G_7 	= 3136,
	G_sh_7 	= 3322,
	A_7 	= 3520,
	A_sh_7 	= 3729,
	B_7 	= 3951,


	/* Octava 8*/
	C_8 	= 4186,
	C_sh_8 	= 4435,
	D_8 	= 4699,
	D_sh_8 	= 4978,
	E_8 	= 5274,
	F_8 	= 5587,
	F_sh_8 	= 5920,
	G_8 	= 6272,
	G_sh_8 	= 6645,
	A_8 	= 7040,
	A_sh_8 	= 7458,
	B_8 	= 7902,

}Frecuency_notes_english_t;


typedef enum{
	mute = 0,
	low = 25,
	play = 50,
	medium = 75,
	full = 100

}sound_level_t;


#endif /* MUSIC_NOTES_H_ */
