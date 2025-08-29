;***************************************************************
;*                 COE 538 FINAL PROJECT                       *
;*               TITLE: EEBOT MAZE SOLVER                      *
;*         BY: Garen Shanoian, Christian turjuman,             *
;*             Gobind Thethi, Gurnoor Bhatoa                   *
;***************************************************************
          
              XDEF Entry, _Startup ;
              ABSENTRY Entry 
              INCLUDE "derivative.inc"

;************************************************
;*        LCD Constants and Equivalents         *
;************************************************
CLEAR_HOME    EQU   $01       ; Clear the display and reset cursor to home
INTERFACE     EQU   $38       ; 8-bit interface, two-line display
CURSOR_OFF    EQU   $0C       ; Enable display, turn cursor off
SHIFT_OFF     EQU   $06       ; Address increments, no character shift
LCD_SEC_LINE  EQU   64        ; Starting address of 2nd LCD line (decimal)

LCD_CNTR      EQU   PTJ       ; LCD control register (E = PJ7, RS = PJ6)
LCD_DAT       EQU   PORTB     ; LCD data register (D7 = PB7 ... D0 = PB0)
LCD_E         EQU   $80       ; Enable signal for LCD
LCD_RS        EQU   $40       ; Register select signal for LCD

NULL          EQU   $00       ; Null character
CR            EQU   $0D       ; Carriage return
SPACE         EQU   ' '       ; Space

;************************************************
;*             State Definitions                *
;************************************************
START         EQU   0          ; Initial state
FWD           EQU   1          ; Forward state
ALL_STP       EQU   2          ; All-stop state
LEFT_TRN      EQU   3          ; Left turn state
RIGHT_TRN     EQU   4          ; Right turn state
REV_TRN       EQU   5          ; Reverse turn state
LEFT_ALIGN    EQU   6          ; Left alignment
RIGHT_ALIGN   EQU   7          ; Right alignment

T_LEFT        EQU   6                             
T_RIGHT       EQU   6                             

;************************************************
;*        Data and Variables Section            *
;************************************************
              ORG   $3800
BASE_LINE     FCB   $9D        ; Baseline calibration for sensors
BASE_BOW      FCB   $CE        ; Front sensor baseline
BASE_MID      FCB   $CE        ; Mid sensor baseline
BASE_PORT     FCB   $CE        ; Port sensor baseline
BASE_STBD     FCB   $CE        ; Starboard sensor baseline

LINE_VARIANCE FCB   $20        ; Line sensor variance
BOW_VARIANCE  FCB   $20        ; Front sensor variance
PORT_VARIANCE FCB   $20        ; Port sensor variance
MID_VARIANCE  FCB   $20        ; Mid sensor variance
STARBOARD_VARIANCE FCB $20     ; Starboard sensor variance

TOP_LINE      RMB   20         ; Top line buffer for LCD
              FCB   NULL
BOT_LINE      RMB   20         ; Bottom line buffer for LCD
              FCB   NULL
CLEAR_LINE    FCC   '                    ' ; Clear buffer
              FCB   NULL
TEMP          RMB   1          ; Temporary storage for port states

;***********************************************
;*             Sensor Storage                  *
;***********************************************

SENSOR_LINE   FCB   $01        ; Line sensor storage
SENSOR_BOW    FCB   $23        ; Front sensor storage
SENSOR_PORT   FCB   $45        ; Port sensor storage
SENSOR_STBD   FCB   $89        ; Starboard sensor storage
SENSOR_MID    FCB   $67        ; Mid sensor storage

SENSOR_NUM    RMB   1          ; Sensor number index

              
              ORG   $3850      ; Where TOF counter register is located
TOF_COUNTER   dc.b  0          ; Timer overflow counter
CRNT_STATE    dc.b  2          ; Current state register
T_TURN        ds.b  1          ; Turn duration
TEN_THOUS     ds.b  1          ; 10,000-digit value
THOUSANDS     ds.b  1          ; 1,000-digit value
HUNDREDS      ds.b  1          ; 100-digit value
TENS          ds.b  1          ; 10-digit value
UNITS         ds.b  1          ; 1-digit value
NO_BLANK      ds.b  1          ; Blanking flag
BCD_SPARE     RMB   2

;************************************************
;*                 Main Code                    *
;************************************************
              ORG   $4000                  ; Set origin of the program
Entry:
_Startup:
              LDS   #$4000                ; Set the stack pointer
              CLI                         ; Enable interrupts
              
              JSR   INIT                  ; Initialize hardware (ports, peripherals)
              JSR   openADC               ; Configure the ADC module
              JSR   initLCD               ; Initialize the LCD display
              JSR   CLR_LCD_BUF           ; Clear LCD buffer
              
              BSET  DDRA, %00000011       ; Configure direction for STAR_DIR and PORT_DIR
              BSET  DDRT, %00110000       ; Configure direction for STAR_SPEED and PORT_SPEED
              
              JSR   initAD                ; Initialize the Analog-to-Digital converter
              
              JSR   initLCD               ; Initialize the LCD  
              JSR   clrLCD                ; Clear LCD and reset cursor position
                 
                                          ; Display initial messages on the LCD
              LDX   #msg1                 ; Load address of the first message
              JSR   putsLCD               ; Display msg1 on the LCD
              LDAA  #$8A                  ; Set cursor position after msg1
              JSR   cmd2LCD               ; Execute LCD command to set cursor position
              LDX   #msg2                 ; Load address of the second message
              JSR   putsLCD               ; Display msg2 on the LCD
              LDAA  #$C0                  ; Move cursor to the second row
              JSR   cmd2LCD               ; Execute LCD command to set cursor position
              LDX   #msg3                 ; Load address of the third message
              JSR   putsLCD               ; Display msg3 on the LCD
              LDAA  #$C7                  ; Set cursor position after msg3
              JSR   cmd2LCD               ; Execute LCD command to set cursor position
              LDX   #msg4                 ; Load address of the fourth message
              JSR   putsLCD               ; Display msg4 on the LCD
              JSR   ENABLE_TOF            ; Enable Time-of-Flight (TOF) initialization

;************************************************
;*               Initial Loop                   *
;************************************************
MAIN
              JSR   G_LEDS_ON             ; Turn on guider LEDs
              JSR   READ_SENSORS          ; Read sensor data
              JSR   G_LEDS_OFF            ; Turn off guider LEDs
              JSR   UPDT_DISPL            ; Update display based on state
              LDAA  CRNT_STATE            ; Load current state into Accumulator A
              JSR   DISPATCHER            ; Call the state dispatcher
              BRA   MAIN                  ; Repeat main loop

;************************************************
;*      Data Section: Messages and States       *
;************************************************
          msg1: dc.b  "St:", 0            ; Label for current state
          msg2: dc.b  "R:", 0             ; Label for sensor readings
          msg3: dc.b  "Vt:", 0            ; Label for battery voltage
          msg4: dc.b  "B:", 0             ; Label for bumper status

          tab:  dc.b  "START  ", 0        ; State labels for debugging
                dc.b  "FWD    ", 0
                dc.b  "ALL_STP", 0
                dc.b  "L_TRN  ", 0
                dc.b  "R_TRN  ", 0
                dc.b  "REVERSE", 0
                dc.b  "L_ALIGN", 0
                dc.b  "R_ALIGN", 0

;************************************************
;*               The Dispatcher                 *
;************************************************
DISPATCHER        CMPA  #START             ; Check if in START state
                  BNE   NOT_START
                  JSR   START_ST           ; Handle START state
                  BRA   DISP_EXIT

NOT_START         CMPA  #FWD               ; Check if in FORWARD state
                  BNE   NOT_FORWARD
                  JSR   FWD_ST             ; Handle FORWARD state
                  BRA   DISP_EXIT

NOT_FORWARD       CMPA  #REV_TRN           ; Check if in REVERSE TURN state
                  BNE   NOT_REV_TURN
                  JSR   REV_TRN_ST         ; Handle REVERSE TURN state
                  BRA   DISP_EXIT

NOT_REV_TURN      CMPA  #ALL_STP           ; Check if in STOP state
                  BNE   NOT_ALL_STOP
                  JSR   ALL_STOP_ST        ; Handle STOP state
                  BRA   DISP_EXIT

NOT_ALL_STOP      CMPA  #LEFT_TRN          ; Check if in LEFT TURN state
                  BNE   NOT_LEFT_TRN
                  JSR   LEFT               ; Handle LEFT TURN state
                  BRA   DISP_EXIT

NOT_LEFT_TRN      CMPA  #LEFT_ALIGN        ; Check if in LEFT ALIGN state
                  BNE   NOT_LEFT_ALIGN
                  JSR   LEFT_ALIGN_DONE    ; Handle LEFT ALIGN state
                  BRA   DISP_EXIT

NOT_LEFT_ALIGN    CMPA  #RIGHT_TRN         ; Check if in RIGHT TURN state
                  BNE   NOT_RIGHT_TRN
                  JSR   RIGHT              ; Handle RIGHT TURN state
                  BRA   DISP_EXIT

NOT_RIGHT_TRN     CMPA  #RIGHT_ALIGN       ; Check if in RIGHT ALIGN state
                  JSR   RIGHT_ALIGN_DONE   ; Handle RIGHT ALIGN state
                  BRA   DISP_EXIT

DISP_EXIT         RTS                      ; Return from dispatcher

;************************************************
;*            START State Handling              *
;************************************************
START_ST
                  BRCLR  PORTAD0, $04, NO_FWD  ; Check if FORWARD BUMP is triggered
                  JSR     INIT_FWD             ; Initialize FORWARD state
                  MOVB    #FWD, CRNT_STATE     ; Set current state to FORWARD
                  BRA     START_EXIT

NO_FWD            NOP                         ; No operation if condition not met
START_EXIT        RTS                         ; Return to MAIN routine

;**********************************************
;*           Forward State (FWD_ST)           *
;**********************************************

FWD_ST            BRSET   PORTAD0, $04, NO_FWD_BUMP           ; Check if front bumper is hit
                  MOVB    #REV_TRN, CRNT_STATE                ; If hit, transition to REVERSE TURN state
                  JSR     UPDT_DISPL                          ; Update display to reflect state change
                  JSR     INIT_REV                            ; Initialize reverse movement
                  LDY     #6000                               ; Delay to ensure smooth operation
                  JSR     del_50us                            
                  JSR     INIT_RIGHT                          ; Initialize right turn
                  LDY     #6000                               ; Delay for action completion
                  JSR     del_50us
                  JMP     FWD_EXIT                            ; Exit forward state

NO_FWD_BUMP       BRSET   PORTAD0, $08, NO_REAR_BUMP          ; Check if rear bumper is hit
                  JSR     INIT_ALL_STP                        ; If hit, stop all movement
                  MOVB    #ALL_STP, CRNT_STATE                ; Transition to ALL STOP state
                  JMP     FWD_EXIT

NO_REAR_BUMP      LDAA    SENSOR_BOW                          ; Check if robot is still on the line
                  ADDA    BOW_VARIANCE
                  CMPA    BASE_BOW
                  BPL     NOT_ALIGNED                         ; If off-line, transition to alignment
                  LDAA    SENSOR_MID                          ; Check middle sensor alignment
                  ADDA    MID_VARIANCE
                  CMPA    BASE_MID
                  BPL     NOT_ALIGNED
                  LDAA    SENSOR_LINE                         ; Check line sensor alignment
                  ADDA    LINE_VARIANCE
                  CMPA    BASE_LINE
                  BPL     CHECK_RIGHT_ALIGN                   ; Check right alignment if needed
                  LDAA    SENSOR_LINE                         ; Evaluate sensor alignment thresholds
                  SUBA    LINE_VARIANCE
                  CMPA    BASE_LINE
                  BMI     CHECK_LEFT_ALIGN                    ; Check left alignment if needed
                  JMP     FWD_EXIT                            ; If aligned, exit forward state

FWD_EXIT          RTS                                         ; Return from forward state

;*********************************************
;           NOT_ALIGNED STATE                *
;*********************************************

NOT_ALIGNED       LDAA    SENSOR_PORT                         ; Check left sensor alignment
                  ADDA    PORT_VARIANCE
                  CMPA    BASE_PORT
                  BPL     PARTIAL_LEFT_TRN                    ; Adjust for partial left turn if misaligned
                  BMI     NO_PORT                             ; Skip adjustment if aligned

NO_PORT           LDAA    SENSOR_BOW                          ; Check front sensor alignment
                  ADDA    BOW_VARIANCE
                  CMPA    BASE_BOW
                  BPL     ALIGNED_EXIT                        ; Exit if aligned
                  BMI     NO_BOW                              ; Otherwise, evaluate further

NO_BOW            LDAA    SENSOR_STBD                         ; Check right sensor alignment
                  ADDA    STARBOARD_VARIANCE
                  CMPA    BASE_STBD
                  BPL     PARTIAL_RIGHT_TRN                   ; Adjust for partial right turn if misaligned
                  BMI     ALIGNED_EXIT                        ; Otherwise, exit if aligned

ALIGNED_EXIT      RTS                                         ; Return from misalignment logic

;*************************************
;          PARTIAL_LEFT_TRN          *
;*************************************

PARTIAL_LEFT_TRN  LDY     #6000                               ; Set delay for left turn duration
                  JSR     del_50us                            
                  JSR     INIT_LEFT                           ; Initialize left turn
                  MOVB    #LEFT_TRN, CRNT_STATE               ; Update state to LEFT TURN
                  LDY     #6000                               ; Set additional delay
                  JSR     del_50us
                  JMP     PARTIAL_LEFT_EXIT                   ; Exit left turn logic

CHECK_LEFT_ALIGN  JSR     INIT_LEFT                           ; Initialize left alignment
                  MOVB    #LEFT_ALIGN, CRNT_STATE             ; Update state to LEFT ALIGN
                  JMP     PARTIAL_LEFT_EXIT

PARTIAL_LEFT_EXIT RTS                                         ; Return from left alignment logic

;*************************************
;           PARTIAL_RIGHT_TRN        *
;*************************************

PARTIAL_RIGHT_TRN LDY     #6000                               ; Set delay for right turn duration
                  JSR     del_50us                            
                  JSR     INIT_RIGHT                          ; Initialize right turn
                  MOVB    #RIGHT_TRN, CRNT_STATE              ; Update state to RIGHT TURN
                  LDY     #6000                               ; Set additional delay
                  JSR     del_50us
                  BRA     PARTIAL_RIGHT_EXIT                  ; Exit right turn logic

CHECK_RIGHT_ALIGN JSR     INIT_RIGHT                          ; Initialize right alignment
                  MOVB    #RIGHT_ALIGN, CRNT_STATE            ; Update state to RIGHT ALIGN
                  BRA     PARTIAL_RIGHT_EXIT

PARTIAL_RIGHT_EXIT RTS                                        ; Return from right alignment logic

;**********************************
;         LEFT and RIGHT          *
;**********************************

LEFT              LDAA    SENSOR_BOW                          ; Check alignment via front sensor
                  ADDA    BOW_VARIANCE
                  CMPA    BASE_BOW
                  BPL     LEFT_ALIGN_DONE                     ; Proceed to left alignment completion
                  JMP     LEFT_RIGHT_EXIT

LEFT_ALIGN_DONE   MOVB    #FWD, CRNT_STATE                    ; Update state to FORWARD
                  JSR     INIT_FWD                            ; Initialize forward movement
                  JMP     LEFT_RIGHT_EXIT

RIGHT             LDAA    SENSOR_BOW                          ; Check alignment via front sensor
                  ADDA    BOW_VARIANCE
                  CMPA    BASE_BOW
                  BPL     RIGHT_ALIGN_DONE                    ; Proceed to right alignment completion
                  JMP     LEFT_RIGHT_EXIT

RIGHT_ALIGN_DONE  MOVB    #FWD, CRNT_STATE                    ; Update state to FORWARD
                  JSR     INIT_FWD                            ; Initialize forward movement
                  JMP     LEFT_RIGHT_EXIT

LEFT_RIGHT_EXIT   RTS                                         ; Return from alignment logic
                                                                                  

;************************************
;*           REV_TRN_ST             *
;************************************

REV_TRN_ST        LDAA    SENSOR_BOW                          ; Read bow sensor value
                  ADDA    BOW_VARIANCE                        ; Add allowable variance
                  CMPA    BASE_BOW                            ; Compare with baseline value
                  BMI     REV_TRN_EXIT                        ; Exit if within acceptable range
                  JSR     INIT_LEFT                           ; Otherwise, initialize left turn to correct alignment
                  MOVB    #FWD, CRNT_STATE                    ; Transition to FORWARD state
                  JSR     INIT_FWD                            ; Initialize forward movement
                  BRA     REV_TRN_EXIT                        ; Exit reverse turning state

REV_TRN_EXIT      RTS                                         ; Return from reverse turning logic

;************************************
;*             ALL_STOP_ST          *
;************************************

ALL_STOP_ST       BRSET   PORTAD0, $04,NO_START               ; Check if forward bumper is triggered
                  JSR     INIT_ALL_STP                        ; Initialize stop (motors off)
                  MOVB    #START,CRNT_STATE                   ; Transition to START state
                  BRA     ALL_STP_EXIT                        ; Exit to MAIN loop

NO_START          NOP                                         ; No operation if condition not met
ALL_STP_EXIT      RTS                                         ; Return from stop state logic

;*****************************************************
;*           Initialization Subroutines              *
;*****************************************************

INIT_RIGHT        BSET    PORTA,%00000010                     ; Set motor for right turn
                  BCLR    PORTA,%00000001                     ; Disable left turn direction
                  LDAA    TOF_COUNTER                         ; Read Time-of-Flight counter
                  ADDA    #T_RIGHT                            ; Add predefined right turn duration
                  STAA    T_TURN                              ; Store turn time
                  RTS                                         ; Return from right turn initialization

INIT_LEFT         BSET    PORTA,%00000001                     ; Set motor for left turn
                  BCLR    PORTA,%00000010                     ; Disable right turn direction
                  LDAA    TOF_COUNTER                         ; Read Time-of-Flight counter
                  ADDA    #T_LEFT                             ; Add predefined left turn duration
                  STAA    T_TURN                              ; Store turn time
                  RTS                                         ; Return from left turn initialization

INIT_FWD          BCLR    PORTA, %00000011                    ; Set motors for forward movement
                  BSET    PTT, %00110000                      ; Enable drive motors
                  RTS                                         ; Return from forward initialization

INIT_REV          BSET    PORTA,%00000011                     ; Set motors for reverse movement
                  BSET    PTT,%00110000                       ; Enable drive motors
                  RTS                                         ; Return from reverse initialization

INIT_ALL_STP      BCLR    PTT, %00110000                      ; Disable drive motors
                  RTS                                         ; Return from stop initialization

;***************************************************************************************************
; INIT: General sensor and port initialization
;***************************************************************************************************

INIT              BCLR    DDRAD,$FF                           ; Configure PORTAD as input (sensor data)
                  BSET    DDRA,$FF                            ; Configure PORTA as output (motor control)
                  BSET    DDRB,$FF                            ; Configure PORTB as output (auxiliary control)
                  BSET    DDRJ,$C0                            ; Configure PTJ pins 6 and 7 as outputs
                  RTS                                         ; Return from initialization

;***************************************************************************************************
; openADC: Configure and initialize the Analog-to-Digital Converter (ADC)
;***************************************************************************************************

openADC           MOVB    #$80,ATDCTL2                       ; Power on the ADC
                  LDY     #1                                 ; Wait for ADC readiness
                  JSR     del_50us                           ; Delay for 50 microseconds
                  MOVB    #$20,ATDCTL3                       ; Configure for 4 conversions on AN1
                  MOVB    #$97,ATDCTL4                       ; Set 8-bit resolution, prescaler=48
                  RTS                                        ; Return from ADC initialization


;********************************************************************************
;*                          Clear LCD Buffer                                    *
;********************************************************************************
; This routine writes characters (ascii 20) into the LCD display
; buffer in order to prepare it for the building of a new display buffer.
; Done only once.
CLR_LCD_BUF       LDX   #CLEAR_LINE
                  LDY   #TOP_LINE
                  JSR   STRCPY

CLB_SECOND        LDX   #CLEAR_LINE
                  LDY   #BOT_LINE
                  JSR   STRCPY

CLB_EXIT          RTS

;*********************************************************************************      
; String Copy
; Copies a null-terminated string (including the null) from one location to another.
; X = starting address of null-terminated string
; Y =  first address of destination
STRCPY            PSHX            ; Protect the registers used
                  PSHY
                  PSHA

STRCPY_LOOP       LDAA 0,X        ; Get a source character
                  STAA 0,Y        ; Copy it to the destination
                  BEQ STRCPY_EXIT ; If it was the null, then exit
                  INX             ; Else increment the pointers
                  INY
                  BRA STRCPY_LOOP ; Repeat steps again in a loop

STRCPY_EXIT       PULA            ; Restore the registers
                  PULY
                  PULX
                  RTS  

;***********************************                                                                       
;*         Guider LEDs ON          *
;***********************************

G_LEDS_ON     BSET PORTA,%00100000 
              RTS

;**********************************
;*         Guider LEDs OFF        *
;**********************************

G_LEDS_OFF    BCLR PORTA,%00100000 
              RTS

;*********************************** 
;*         Read Sensors            *
;***********************************


READ_SENSORS  CLR SENSOR_NUM ; Select sensor number 0
              LDX #SENSOR_LINE ; Point at the start of the sensor array

RS_MAIN_LOOP  LDAA SENSOR_NUM ; Select the correct sensor input
              JSR SELECT_SENSOR ; on the hardware
              LDY #400 ; 20 ms delay to allow the
              JSR del_50us ; sensor to stabilize

              LDAA #%10000001 ; Start A/D conversion on AN1
              STAA ATDCTL5
              BRCLR ATDSTAT0,$80,* ; Repeat until A/D signals done

              LDAA ATDDR0L ; A/D conversion is complete in ATDDR0L
              STAA 0,X ; so copy it to the sensor register
              CPX #SENSOR_STBD ; If this is the last reading
              BEQ RS_EXIT ; Then exit

              INC SENSOR_NUM ; Else, increment the sensor number
              INX ; and the pointer into the sensor array
              BRA RS_MAIN_LOOP ; and do it again

RS_EXIT       RTS


;**********************************
;*          Select Sensor         *
;**********************************

SELECT_SENSOR PSHA ; Save the sensor number for the moment
            
            LDAA PORTA ; Clear the sensor selection bits to zeros
            ANDA #%11100011 ;
            STAA TEMP ; and save it into TEMP
            
            PULA ; Get the sensor number
            ASLA ; Shift the selection number left, twice
            ASLA ;
            ANDA #%00011100 ; Clear irrelevant bit positions
            
            ORAA TEMP ; OR it into the sensor bit positions
            STAA PORTA ; Update the hardware
            RTS

;**********************************
;*     Display Sensor Readings    *
;**********************************

DP_FRONT_SENSOR EQU TOP_LINE+3
DP_PORT_SENSOR  EQU BOT_LINE+0
DP_MID_SENSOR   EQU BOT_LINE+3
DP_STBD_SENSOR  EQU BOT_LINE+6
DP_LINE_SENSOR  EQU BOT_LINE+9

DISPLAY_SENSORS LDAA SENSOR_BOW ; Get the FRONT sensor value
                JSR BIN2ASC ; Convert to ascii string in D
                LDX #DP_FRONT_SENSOR ; Point to the LCD buffer position
                STD 0,X ; and write the 2 ascii digits there

                LDAA SENSOR_PORT ; Repeat for the PORT value
                JSR BIN2ASC
                LDX #DP_PORT_SENSOR
                STD 0,X

                LDAA SENSOR_MID ; Repeat for the MID value
                JSR BIN2ASC
                LDX #DP_MID_SENSOR
                STD 0,X

                LDAA SENSOR_STBD ; Repeat for the STARBOARD value
                JSR BIN2ASC
                LDX #DP_STBD_SENSOR
                STD 0,X

                LDAA SENSOR_LINE ; Repeat for the LINE value
                JSR BIN2ASC
                LDX #DP_LINE_SENSOR
                STD 0,X

                LDAA #CLEAR_HOME ; Clear the display and home the cursor
                JSR cmd2LCD ; "

                LDY #40 ; Wait 2 ms until "clear display" command is complete
                JSR del_50us

                LDX #TOP_LINE ; Now copy the buffer top line to the LCD
                JSR putsLCD

                LDAA #LCD_SEC_LINE ; Position the LCD cursor on the second line
                JSR LCD_POS_CRSR

                LDX #BOT_LINE ; Copy the buffer bottom line to the LCD
                JSR putsLCD
                RTS
                

;********************************************************************************************
;* Update Display (Current State + Bumper Switches + Battery Voltage + Sensor Readings)     *
;********************************************************************************************
UPDT_DISPL      LDAA  #$82                      ; Move LCD cursor to the end of msg1
                JSR   cmd2LCD                   ;
                
                LDAB  CRNT_STATE                ; Display current state
                LSLB                            ; "
                LSLB                            ; "
                LSLB                            ; "
                LDX   #tab                      ; "
                ABX                             ; "
                JSR   putsLCD                   ; "
;*******************************************************************************************               
                LDAA  #$8F                      ; Move LCD cursor to the end of msg2
                JSR   cmd2LCD                   ; ""
                LDAA  SENSOR_BOW                ; Convert value from SENSOR_BOW to a
                JSR   BIN2ASC                   ; Two digit hexidecimal value
                JSR   putcLCD                   ; ""
                EXG   A,B                       ; ""
                JSR   putcLCD                   ; ""

                LDAA  #$92                      ; Move LCD cursor to Line position 
                JSR   cmd2LCD                   ; ""
                LDAA  SENSOR_LINE               ; Convert value from SENSOR_LINE to a
                JSR   BIN2ASC                   ; Two digit hexidecimal value
                JSR   putcLCD                   ; ""
                EXG   A,B                       ; ""
                JSR   putcLCD                   ; ""

                LDAA  #$CC                      ; Move LCD cursor to Port position on 2nd row 
                JSR   cmd2LCD                   ; ""
                LDAA  SENSOR_PORT               ; Convert value from SENSOR_PORT to a
                JSR   BIN2ASC                   ; Two digit hexidecimal value
                JSR   putcLCD                   ; ""
                EXG   A,B                       ; ""
                JSR   putcLCD                   ; ""

                LDAA  #$CF                      ; Move LCD cursor to Mid position on 2nd row 
                JSR   cmd2LCD                   ; ""
                LDAA  SENSOR_MID                ; Convert value from SENSOR_MID to a
                JSR   BIN2ASC                   ; Two digit hexidecimal value
                JSR   putcLCD                   ; ""
                EXG   A,B                       ; ""
                JSR   putcLCD                   ; ""

                LDAA  #$D2                      ; Move LCD cursor to Starboard position on 2nd row 
                JSR   cmd2LCD                   ; ""
                LDAA  SENSOR_STBD               ; Convert value from SENSOR_STBD to a
                JSR   BIN2ASC                   ; Two digit hexidecimal value
                JSR   putcLCD                   ; ""
                EXG   A,B                       ; ""
                JSR   putcLCD                   ; ""
;********************************************************************************************          
                MOVB  #$90,ATDCTL5              ; Uns., sing. conv., mult., ch=0, start
                BRCLR ATDSTAT0,$80,*            ; Wait until the conver. seq. is complete
                LDAA  ATDDR0L                   ; Load the ch0 result - battery volt - into A
                LDAB  #39                       ; AccB = 39
                MUL                             ; AccD = 1st result x 39
                ADDD  #600                      ; AccD = 1st result x 39 + 600
                JSR   int2BCD
                JSR   BCD2ASC
                LDAA  #$C2                      ; move LCD cursor to the end of msg3
                JSR   cmd2LCD                   ; "                
                LDAA  TEN_THOUS                 ; output the TEN_THOUS ASCII character
                JSR   putcLCD                   ; "
                LDAA  THOUSANDS                 ; output the THOUSANDS ASCII character
                JSR   putcLCD                   ; "
                LDAA  #$2E                      ; output the HUNDREDS ASCII character
                JSR   putcLCD                   ; "
                LDAA  HUNDREDS                  ; output the HUNDREDS ASCII character
                JSR   putcLCD                   ; "                
;********************************************************************************************
                LDAA  #$C9                      ; Move LCD cursor to the end of msg4
                JSR   cmd2LCD
                
                BRCLR PORTAD0,#%00000100,bowON  ; If FWD_BUMP, then
                LDAA  #$20                      ;
                JSR   putcLCD                   ;
                BRA   stern_bump                ; Display 'B' on LCD (Bumper state)
         bowON: LDAA  #$42                      ; ""
                JSR   putcLCD                   ; ""
          
    stern_bump: BRCLR PORTAD0,#%00001000,sternON; If REV_BUMP, then
                LDAA  #$20                      ;
                JSR   putcLCD                   ;
                BRA   UPDT_DISPL_EXIT           ; Display 'S' on LCD
       sternON: LDAA  #$53                      ; ""
                JSR   putcLCD                   ; ""
UPDT_DISPL_EXIT RTS                             ; and exit
                
;***************************************************************************************************
;***************************************************************************************************
ENABLE_TOF        LDAA    #%10000000
                  STAA    TSCR1           ; Enable TCNT
                  STAA    TFLG2           ; Clear TOF
                  LDAA    #%10000100      ; Enable TOI and select prescale factor equal to 16
                  STAA    TSCR2
                  RTS

TOF_ISR           INC     TOF_COUNTER
                  LDAA    #%10000000      ; Clear
                  STAA    TFLG2           ; TOF
                  RTI


; Subroutines for Utilities (LCD + Delay + Integer to Binanry etc.)
;***************************************************************************************************
initLCD:          BSET    DDRB,%11111111  ; configure pins PS7,PS6,PS5,PS4 for output
                  BSET    DDRJ,%11000000  ; configure pins PE7,PE4 for output
                  LDY     #2000
                  JSR     del_50us
                  LDAA    #$28
                  JSR     cmd2LCD
                  LDAA    #$0C
                  JSR     cmd2LCD
                  LDAA    #$06
                  JSR     cmd2LCD
                  RTS
;***************************************************************************************************
cmd2LCD:          BCLR  LCD_CNTR, LCD_RS ; select the LCD instruction
                  JSR   dataMov          ; send data to IR
                  RTS

;***************************************************************************************************
putsLCD:          LDAA  1,X+             ; get one character from  string
                  BEQ   donePS           ; get NULL character
                  JSR   putcLCD
                  BRA   putsLCD

donePS            RTS

;***************************************************************************************************

clrLCD:           LDAA  #$01
                  JSR   cmd2LCD
                  LDY   #40
                  JSR   del_50us
                  RTS

;***************************************************************************************************
del_50us          PSHX                   ; (2 E-clk) Protect the X register
eloop             LDX   #300             ; (2 E-clk) Initialize the inner loop counter
iloop             NOP                    ; (1 E-clk) No operation
                  DBNE X,iloop           ; (3 E-clk) If the inner cntr not 0, loop again
                  DBNE Y,eloop           ; (3 E-clk) If the outer cntr not 0, loop again
                  PULX                   ; (3 E-clk) Restore the X register
                  RTS                    ; (5 E-clk) Else return


;***************************************************************************************************
putcLCD:          BSET  LCD_CNTR, LCD_RS  ; select the LCD data register (DR)c
                  JSR   dataMov           ; send data to DR
                  RTS

;***************************************************************************************************
dataMov:          BSET  LCD_CNTR, LCD_E   ; pull LCD E-signal high
                  STAA  LCD_DAT           ; send the upper 4 bits of data to LCD
                  BCLR  LCD_CNTR, LCD_E   ; pull the LCD E-signal low to complete write oper.
                  LSLA                    ; match the lower 4 bits with LCD data pins
                  LSLA                    ; ""
                  LSLA                    ; ""
                  LSLA                    ; ""
                  BSET  LCD_CNTR, LCD_E   ; pull LCD E-signal high
                  STAA  LCD_DAT           ; send the lower 4 bits of data to LCD
                  BCLR  LCD_CNTR, LCD_E   ; pull the LCD E-signal low to complete write oper.
                  LDY   #1                ; adding this delay allows
                  JSR   del_50us          ; completion of most instructions
                  RTS

;***************************************************************************************************
initAD            MOVB  #$C0,ATDCTL2      ;power up AD, select fast flag clear
                  JSR   del_50us          ;wait for 50 us
                  MOVB  #$00,ATDCTL3      ;8 conversions in a sequence
                  MOVB  #$85,ATDCTL4      ;res=8, conv-clks=2, prescal=12
                  BSET  ATDDIEN,$0C       ;configure pins AN03,AN02 as digital inputs
                  RTS

;***************************************************************************************************
int2BCD           XGDX                    ;Save the binary number into .X
                  LDAA #0                 ;Clear the BCD_BUFFER
                  STAA TEN_THOUS
                  STAA THOUSANDS
                  STAA HUNDREDS
                  STAA TENS
                  STAA UNITS
                  STAA BCD_SPARE
                  STAA BCD_SPARE+1
                  CPX #0                  ; Check for a zero input
                  BEQ CON_EXIT            ; and if so, exit
                  XGDX                    ; Not zero, get the binary number back to .D as dividend
                  LDX #10                 ; Setup 10 (Decimal!) as the divisor
                  IDIV                    ; Divide Quotient is now in .X, remainder in .D
                  STAB UNITS              ; Store remainder
                  CPX #0                  ; If quotient is zero,
                  BEQ CON_EXIT            ; then exit
                  XGDX                    ; else swap first quotient back into .D
                  LDX #10                 ; and setup for another divide by 10
                  IDIV
                  STAB TENS
                  CPX #0
                  BEQ CON_EXIT
                  XGDX                    ; Swap quotient back into .D
                  LDX #10                 ; and setup for another divide by 10
                  IDIV
                  STAB HUNDREDS
                  CPX #0
                  BEQ CON_EXIT
                  XGDX                    ; Swap quotient back into .D
                  LDX #10                 ; and setup for another divide by 10
                  IDIV
                  STAB THOUSANDS
                  CPX #0
                  BEQ CON_EXIT
                  XGDX                    ; Swap quotient back into .D
                  LDX #10                 ; and setup for another divide by 10
                  IDIV
                  STAB TEN_THOUS

CON_EXIT          RTS                     ; Were done the conversion

LCD_POS_CRSR      ORAA #%10000000         ; Set the high bit of the control word
                  JSR cmd2LCD             ; and set the cursor address
                  RTS

;************************* converts a 8-bit binary number in ACCA into its ASCII representation********************************************
HEX_TABLE             FCC   '0123456789ABCDEF'    ; Table for converting values

BIN2ASC               PSHA               ; Save a copy of the input number
                      TAB            
                      ANDB #%00001111     ; Strip off the upper nibble
                      CLRA                ; D now contains 000n where n is the Lower nibble. Clear accumulator A.
                      ADDD #HEX_TABLE     ; Set up for indexed load
                      XGDX                
                      LDAA 0,X            ; Get the Lower nibble character
                      
                      PULB                ; Retrieve the input number into ACCB
                      PSHA                ; and push the Lower nibble character in its place
                      RORB                ; Move the upper nibble of the input number
                      RORB                ; into the lower nibble position.
                      RORB
                      RORB 
                      ANDB #%00001111     ; Strip off the upper nibble
                      CLRA                ; D now contains 000n where n is the Upper Snibble 
                      ADDD #HEX_TABLE     ; Set up for indexed load
                      XGDX                                                               
                      LDAA 0,X            ; Get the Upper Snibble character into ACCA
                      PULB                ; Retrieve the Lower Snibble character into ACCB
                      RTS                                        
                      
;***************************************************************************************
;                                Integer to BCD                                        *
;***************************************************************************************

BCD2ASC       LDAA  #0                        ; Initialize the blanking flag
              STAA  NO_BLANK

C_TTHOU       LDAA  TEN_THOUS                 ; Check the ?ten_thousands? digit
              ORAA  NO_BLANK
              BNE   NOT_BLANK1

ISBLANK1      LDAA  #' '                      ; It?s blank
              STAA  TEN_THOUS                 ; so store a space
              BRA   C_THOU                    ; and check the ?thousands? digit

NOT_BLANK1    LDAA  TEN_THOUS                 ; Get the ?ten_thousands? digit
              ORAA  #$30                      ; Convert to ascii
              STAA  TEN_THOUS
              LDAA  #$1                       ; Signal that we have seen a ?non-blank? digit
              STAA  NO_BLANK

C_THOU        LDAA  THOUSANDS                 ; Check the thousands digit for blankness
              ORAA  NO_BLANK                  ; If it?s blank and ?no-blank? is still zero
              BNE   NOT_BLANK2

ISBLANK2      LDAA  #' '                      ; Thousands digit is blank
              STAA  THOUSANDS                 ; so store a space
              BRA   C_HUNS                    ; and check the hundreds digit

NOT_BLANK2    LDAA  THOUSANDS                 ; (similar to ?ten_thousands? case)
              ORAA  #$30
              STAA  THOUSANDS
              LDAA  #$1
              STAA  NO_BLANK

C_HUNS        LDAA  HUNDREDS                  ; Check the hundreds digit for blankness
              ORAA  NO_BLANK                  ; If it?s blank and ?no-blank? is still zero
              BNE   NOT_BLANK3

ISBLANK3      LDAA  #' '                      ; Hundreds digit is blank
              STAA  HUNDREDS                  ; so store a space
              BRA   C_TENS                    ; and check the tens digit

NOT_BLANK3    LDAA  HUNDREDS                  ; (similar to ?ten_thousands? case)
              ORAA  #$30
              STAA  HUNDREDS
              LDAA  #$1
              STAA  NO_BLANK

C_TENS        LDAA  TENS                      ; Check the tens digit for blankness
              ORAA  NO_BLANK                  ; If it?s blank and ?no-blank? is still zero
              BNE   NOT_BLANK4

ISBLANK4      LDAA  #' '                      ; Tens digit is blank
              STAA  TENS                      ; so store a space
              BRA   C_UNITS                   ; and check the units digit

NOT_BLANK4    LDAA  TENS                      ; (similar to ?ten_thousands? case)
              ORAA  #$30
              STAA  TENS

C_UNITS       LDAA  UNITS                     ; No blank check necessary, convert to ascii.
              ORAA  #$30
              STAA  UNITS

              RTS                             ; Completed


;***************************************************************************************************
;*                                Interrupt Vectors                                                *
;***************************************************************************************************
              ORG     $FFFE
              DC.W    Entry ; Reset Vector
              ORG     $FFDE
              DC.W    TOF_ISR ; Timer Overflow Interrupt Vector


