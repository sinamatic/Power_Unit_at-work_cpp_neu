# software
- install STM32CubeIDE
- select the Microchip (F303CCT)
- Select prefered language (C or CPP)
- Generate Code
- Make Hardware Configuration (look in circuit plan which pin is used for what, label pins)

# Hardware Config
- Right click on Pin to change the name and add USER LABEL, e.g. STM_ARM_SENSE (instead of "PBxyADCxy")
- ADC's need to be cycled, you have to know which is which, so read one after another
- Make the following Settings for ADCs

## Parameter Settings
- Mode: Independent mode
- Clock Prescaler: Synconous clock mode divided by [Number of channels, if number is 3 then choose 4]
- Scan Conversion Mode: Enabled [only possible if number of channels > 1]
- Continuous Conversion: Enabled
- Discontinuous Conversion: Disabled
- DMA Continuous Requests: Enabled [if not possible, check out and update DMA Settings first]
- End of Conversion Sequence: End of sequence of conversion
- Overrun behaviour: Overrun data preserved
- Low Power Auto Wait: Disabled

- Enable Regular Conversion: Enable
- Number of Conversion [Number of Channels]
- For Every Channel add a Rank and set it, for example ADC1 Rank 1 = Channel 1, ADC1 Rank 2 = Channel 4 
- Sampling Time 181.5 Cycles (faster is not needed for PowerUnit in this case, makes the average of data automatically, the slower the better the calculation, for further infos ask Prof. May)

## DMA Settings
- ADD
- Channel should be selected automatically (only one choosable)
- Mode: Circular
- Just Select "Memory"
- Data with: Peripheral: Half Word, Memory: Half Word

## NVIC Settings
Enable Everything, especially interrupts


# ADCs Cyclen
- create Buffer in USER CODE BEGIN PV, Number of Buffer = Number of ADCs, Size of Buffer = Number of Channels of this ADC
- In this case, number of ADCs: 4, Number of Channels of ADC1 = 2, ADC2 = 2, ADC3 = 1, ADC4 = 2:

hadc1_number_of_channels = 2;
hadc2_number_of_channels = 2;
hadc3_number_of_channels = 1;
hadc4_number_of_channels = 2;

## How to create buffer
```c
/* USER CODE BEGIN PV */

//Buffer für Anzahl an Channels
volatile uint16_t ADC1_DMA_BUFFER[2];		// Buffer IN1, IN4 
volatile uint16_t ADC2_DMA_BUFFER[2];		// Buffer IN1, IN3 
volatile uint16_t ADC3_DMA_BUFFER[1];		// Buffer IN1 
volatile uint16_t ADC4_DMA_BUFFER[2];		// Buffer IN3, IN4 

/* USER CODE END PV */
```

# write own functions
- don't forget constructor in the beginning

```c
/* USER CODE BEGIN PFP */

void enable_Arm(void);
void disable_Arm(void);
void enable_Drive(void);
void disable_Drive(void);


/* USER CODE END PFP */
```

# Read ADCs
- start the first ADC in main function `int main(void)`, then all the others start with interrupfs
- directly before `while(1)` start ADC1

```c
HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &ADC1_DMA_BUFFER, 2);
```

- then create function to cycle all adcs in the function `void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)`
- the function reads one ADC-Value, writes it in buffer, saves it in variable, reads the next one
- after that, the Next ADC should be startet (inside the if or it doesn't work)
- first ADC is swiched on in main-function, so the next start should be after the last ADC


```c
/* USER CODE BEGIN 4 */


uint16_t adc1_rank1_intern_sense_data = 420;
uint16_t adc1_rank2_12v_sense_data = 0;
uint16_t adc2_rank1_drive_1_sense_data = 0; //nicht benötigt
uint16_t adc2_rank2_5v_sense_data = 0;
uint16_t adc3_drive_2_sense_data = 0;
uint16_t adc4_rank1_arm_sense_data = 0;
uint16_t adc4_rank2_48v_sense_data = 420; //y


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{

	int hadc1_number_of_channels = 2;
	int hadc2_number_of_channels = 2;
	int hadc3_number_of_channels = 1;
	int hadc4_number_of_channels = 2;

	HAL_ADC_Stop_DMA(hadc); //Stop running ADCs
	//HAL_Delay(10);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &ADC1_DMA_BUFFER, hadc1_number_of_channels); //Start ADC1
	//HAL_Delay(10);

	/*
	 * Reading ADC 1
	 */

	if (hadc == &hadc1) {
		adc1_rank1_intern_sense_data = ADC1_DMA_BUFFER[0]; //ToDo: Formel, direkt konvertieren
		adc1_rank2_12v_sense_data = ADC1_DMA_BUFFER[1]; //ToDo: Formel, direkt konvertieren
		HAL_ADC_Start_DMA(&hadc2, (uint32_t*) &ADC2_DMA_BUFFER, hadc2_number_of_channels); //Start ADC2
	}

	/*
	 * Reading ADC 2
	 */

	if (hadc == &hadc2) {
		adc2_rank1_drive_1_sense_data = ADC2_DMA_BUFFER[0]; //ToDo: Formel, direkt konvertieren
		adc2_rank2_5v_sense_data = ADC2_DMA_BUFFER[1]; //ToDo: Formel, direkt konvertieren
		HAL_ADC_Start_DMA(&hadc3, (uint32_t*) &ADC3_DMA_BUFFER, hadc3_number_of_channels); //Start ADC3
	}


	/*
	 * Reading ADC 3
	 */

	if (hadc == &hadc3) {
		adc3_drive_2_sense_data = ADC3_DMA_BUFFER[0]; //ToDo: Formel, direkt konvertieren
		HAL_ADC_Start_DMA(&hadc4, (uint32_t*) &ADC4_DMA_BUFFER, hadc4_number_of_channels); //Start ADC4
	}


	/*
	 * Reading ADC 4
	 */

	if (hadc == &hadc4) {
		adc4_rank1_arm_sense_data = ADC4_DMA_BUFFER[0]; //ToDo: Formel, direkt konvertieren
		adc4_rank2_48v_sense_data = ADC4_DMA_BUFFER[1]; //ToDo: Formel, direkt konvertieren
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &ADC1_DMA_BUFFER, hadc1_number_of_channels); //Start ADC1
	}

}
```

# set MOSFETs
```c
/* USER CODE BEGIN 4 */

/* ************************************************************************************************
 *
 * Name: void enable_Arm(void), void disable_Arm(void),
 *
 * Description: enables or disables the arm
 *
 * ***********************************************************************************************/

bool arm_enabled;

void enable_Arm(void){
	HAL_GPIO_WritePin(STM_ARM_SW_GPIO_Port, STM_ARM_SW_Pin, GPIO_PIN_SET);
	arm_enabled = true;
}


void disable_Arm(void){
	HAL_GPIO_WritePin(STM_ARM_SW_GPIO_Port, STM_ARM_SW_Pin, GPIO_PIN_RESET);
	arm_enabled = false;
}


/* ************************************************************************************************
 *
 * Name: void enable_Drive(void), void disable_Drive(void),
 *
 * Description: enables or disables the Drive
 *
 * ***********************************************************************************************/

bool drive_enabled;

void enable_Drive(void){
	HAL_GPIO_WritePin(STM_DRIVE_2_SW_GPIO_Port, STM_DRIVE_2_SW_Pin, GPIO_PIN_SET);
	drive_enabled = true;
}


void disable_Drive(void){
	HAL_GPIO_WritePin(STM_DRIVE_2_SW_GPIO_Port, STM_DRIVE_2_SW_Pin, GPIO_PIN_RESET);
	drive_enabled = false;
}

/* USER CODE END 4*/
```

# USB Connection
- in Hardware Config Activate USB in Connectivity
- check everything in following window (folder USB_DEVICE should be generated)
- USB_DEVICE/App/usbd_cdc_if.c change functions at the bottom to the following, for further details ask Niklas Kohlisch (2023, Rescue)

```c
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
```

# STM32CubeIDE
## Build
- Symbol of hammer in settings line in the top right corner

## Debug
- with doubleclick on line number -> set breakpoint
- Run / Debug as / STM32CubeIDE Project 
- switch to debugger (you need to connect the laptop with a programming adapter to the power unit, fix it with a rubber or put something on it so it's fixed
- with the "step forward" button you can step from breakpoint to breakpoint
- if all breakpoints are removed (doubleclick on breakpoint/line number to set/reset it) the code runs through
- to watch live expression, open window (right, click though palettes)
- copy + paste name of variable in window 

## Flash
- open build-menue (down-arrow next to hammer-symbol)
- choose "release"
- click on flash sign (left next to Hammer-Symbol)

# Bugs
- if "enable_Arm" and/or "enable_drive" in main, while, Conv_Callback is used, ADC-Values can not be read and debugger crashes after few seconds
- if arm and drive are disabled, ADC-values can be read
- ADC1 rank 1 (intern) sould give a value > 0, doesnt work, but changes from 420 to 0 after start so anything happens

# to do
- convert ADC values to usedful data
- make buzzer beep if battery is emptry (48V value under treashold) 
- shutdown arm/drive/robot if battery is empty
- show data on display (USB connection, rosserial?)



