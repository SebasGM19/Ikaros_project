/*
 * lcd.c
 *
 *  Created on: Jun 12, 2024
 *      Author: sebas
 */

#include "lcd.h"

uint8_t volatile coordinate_X = 0;//Coordinates
uint8_t volatile coordinate_Y = 0;

uint8_t letraA = 0b01000001;

//todos estos comandos se deben enviar con el rs en 0 espera minima de 5ms en cada indicacion excepto cuando se energiza (100ms)
#define SET_8_BITS_MODE				(0x30) //indica que se quiere configurar 3
#define SET_4_BITS_CONFIGURATION	(0x20) //4BITS 2LINES, 5x8
#define SET_2_LINES_QUALITY_5X8		(0x28)
#define START_LCD_WITHOUT_CURSOR	(0x0C)
#define CLEAN_SCREEN		 		(0x01)
//then indicate the position, you should indicate the position to start
//the function lcd_print() se utilizara para solo mostrar en la primera linea y cada vez se b¿r

//comandos de posicion
#define RETURN_HOME						(0x02) //vuelve al inicio (0,0)
#define MOVE_CURSOS_TO_LEFT				(0x10)
#define MOVE_CURSOR_TO_RIGHT			(0x14)
#define MOVE_DISPLAY_TO_LEFT			(0x18) //mueve la escritua de la pantalla a la izquierda
#define MOVE_DISPLAY_TO_RIGHT			(0x1C) //mueve la escritur de la pantalla a la derecha

//comandos de posicion altura
#define START_AT_ROW1	(0x00)  //para definir la posicion de debe de mandar el comando de donde desea iniciar
#define START_AT_ROW2	(0x40)
#define START_AT_ROW3	(0x10)
#define START_AT_ROW4	(0x50)

//NOTA
/*
 * ROW 1 start at 0x00 and end in 0x0F it meand is map from = 0 to 15 in Xaxis of the firts row(Yaxis 0)
 */

//para definir la posicion de debe de mandar el comando de donde desea iniciar
/*
 * se debera definir el inicio de la escritura con la posicion 0x80|START_AT_ROWX = comando a mandar
 * ejemplo:
 * 0x80|0x40 = 0xC0 este comando sera enviado despues de aplicar el comando CLEAN_SCREEN
 */

controls_gpios_t volatile control_alternative;

void set_new_coordinates(uint8_t new_X_value, uint8_t new_Y_value){
	coordinate_X = new_X_value;
	coordinate_Y = new_Y_value;
}

void get_actual_coordinates(uint8_t X_position,uint8_t Y_position){
	X_position = coordinate_X;
	Y_position = coordinate_Y;
}

void set_controls_gpios(Pin_number_t RS, Pin_number_t RW, Pin_number_t E,Set_Port_t PORT){
	control_alternative.PIN_RS = RS;
	control_alternative.PIN_RW = RW;
	control_alternative.PIN_E = E;
	control_alternative.PIN_D4 = (E+1);
	control_alternative.PORT = PORT;

}


/*
 * para enviar un comando se separa en 4 bits el ultimo bit de los MSB correspondera al menor GPIO
 * Es decir si es configuracion B el PIN_7 sera al que le corresponde el ultimo bit de los 4
 *
 * Example: 0001 1001
 * 0001
 * PIN_10 = 0, PIN_9 =0, PIN_8 = 0, PIN_7 = 1
 *
 * 1001
 * PIN_10 = 1, PIN_9 =0, PIN_8 = 0, PIN_7 = 1
 *
 */
Status_code_t Set_command(uint8_t command, command_type_t type){
	uint8_t MSB = 0;
	uint8_t LSB = 0;

	GPIO_DigitalWrite(control_alternative.PORT, control_alternative.PIN_RS, type);

	MSB = (command >> 4);
	LSB = (command & 0x0F); 	//keep only the first 4 bits

	for(uint8_t i =0; i<3;i++){
		GPIO_DigitalWrite(control_alternative.PORT, (control_alternative.PIN_D4 + i), (MSB&1));
		MSB = MSB>>1;
	}
	Delay(100);
	GPIO_DigitalWrite(control_alternative.PORT, control_alternative.PIN_E, Low);
	Delay(100);


	for(uint8_t i =0; i<3;i++){
		GPIO_DigitalWrite(control_alternative.PORT, (control_alternative.PIN_D4 + i), (LSB&1));
		LSB = LSB>>1;
	}
	GPIO_DigitalWrite(control_alternative.PORT, control_alternative.PIN_E, High);
	Delay(100);
	GPIO_DigitalWrite(control_alternative.PORT, control_alternative.PIN_E, Low);
	Delay(100);

	return Success;

}

Status_code_t Init_lcd(lcd_alternative_t lcd_alternative){

	//EN tODo EL INIT DEBE DE ESTAR RS EN 0 , CUANDO PASE A ESCRITURA SE DEBERA PASAR A 1
	uint16_t volatile PositionsOfPin =0;

	switch(lcd_alternative){
	case lcd_PortB:
		set_controls_gpios(Pin_4, Pin_5, Pin_6, Port_B);
		break;
	case lcd_PortC:
		set_controls_gpios(Pin_7, Pin_8, Pin_9, Port_C);
		break;
	default:
		return OptionNotSupported;
	}

	uint32_t volatile *pPort_ModeReg = (uint32_t volatile *)(control_alternative.PORT+ OFFSET_PORTS);
	ClockEnable(control_alternative.PORT, Enabled);

	PositionsOfPin = (uint16_t volatile)(lcd_alternative * 2);
	*pPort_ModeReg &= ~(clear_fourteen_bits<<PositionsOfPin);
	*pPort_ModeReg |= (0x1555 << PositionsOfPin);// equal to : 01 0101 0101 0101 set as output

	GPIO_DigitalWrite(control_alternative.PORT, control_alternative.PIN_RW, Low); //this stay LOW in 4bit configuration

	GPIO_DigitalWrite(control_alternative.PORT, control_alternative.PIN_RS, Low);
	GPIO_DigitalWrite(control_alternative.PORT, control_alternative.PIN_E, Low);

	/*set to low all gpio*/
	for(uint8_t i =0; i<4; i++){
		GPIO_DigitalWrite(control_alternative.PORT, (lcd_alternative+i), Low);
	}

	Delay(100000);
	Set_command(SET_8_BITS_MODE, set_command);
	Delay(5000);
	Set_command(SET_8_BITS_MODE, set_command);
	Delay(5000);
	Set_command(SET_8_BITS_MODE, set_command);
	Delay(5000);
	Set_command(SET_4_BITS_CONFIGURATION, set_command);
	Delay(5000);
	Set_command(SET_2_LINES_QUALITY_5X8, set_command);
	Delay(5000);
	Set_command(START_LCD_WITHOUT_CURSOR, set_command);
	Delay(5000);
	Set_command(CLEAN_SCREEN, set_command);
	Delay(5000);

	return Success;
}

/*en esta funcion solo se podra escribir en la primera linea, cada vez que se escriba se limpiara la pantalla
 * volvera a la posicion inicial y despues escribira nuevamente el mensaje
 */
void lcd_print(uint8_t* data, uint8_t *data_lenght){
//aqui no se seleccionara la posicion, solo imprimira en donde sea que este que al encender es la posicion 0,0

}


/*
 * en esta funcion se debera elegir cada vez en que posicion se quera escribir, en coordenada x y y, estara mas completa
 * en esta funcion no se borrara nada de infor, solo se sustituira, a menos que se ingrese la funcion CLENA_SCREEN
 */
void lcd_printXY(uint8_t X_axis, uint8_t Y_axis, uint8_t* data, uint8_t *data_lenght){
	//despues de las configuraciones imprimir
	lcd_print(data,data_lenght);
}

//limpia la pantalla
void lcd_clean_screen(void){

}

//vuelve a la posicion 0,0
void lcd_return_to_home(void){

}


