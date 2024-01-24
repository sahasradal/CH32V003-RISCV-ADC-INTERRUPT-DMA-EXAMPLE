# Works fine , PD4 and PD3 are ADC input, PD5 & PD6 = TX and RX, prints to terminal
#BRR = BAUD  
#13.6=38400
#4.4 = 115200
#52.1 = 9600
#8.7 =  57600
#USARTx_TX Full-duplex mode Push-pull multiplexed outputs
#USARTx_RX Full-duplex mode Floating input or pull-up input
#PD5 -tx
#PD6 -rx
#PA2 = A0
#PA1 = A1
#PC4 = A2
#PD2 = A3
#PD3 = A4
#PD4 = A7


fclk = 24000000   # 24Mhz RCO internal , AHB =8Mhz by default
buffer0 = 0x20000004
statusreg = 0x20000004
buffer1 = 0x20000008
result_low = 0x20000004
result_hi = 0x20000008
state = 0x2000000C
result1 = 0x20000010
result2 = 0x20000014
dividend = 0x20000018 
divisor = 0x2000001C
scratch = 0x20000020
mem = 0x20000024

include CH32V003_reg1.asm

vtable:
	j reset_handler		#  longs 0x00000000 # RESERVED 0
align 4
  longs   0x00000000 # RESERVED 1
  longs   0x00000000 #pack <l longs NMI_IRQhandler
  longs   0x00000000 #pack <l HardFault_IRQhandler
  longs   0x00000000 # RESERVED 4
  longs   0x00000000 # RESERVED 5
  longs   0x00000000 # RESERVED 6
  longs   0x00000000 # RESERVED 7
  longs   0x00000000 # RESERVED 8
  longs   0x00000000 # RESERVED 9
  longs   0x00000000 # RESERVED 10
  longs   0x00000000 # RESERVED 11
  longs   0x00000000 # pack <l SysTick_IRQhandler	#; place the address of the mtime ISR subroutine in the vector table position 7,assembler will store isr address here, longs 0x00000000 # RESERVED 12	
  longs   0x00000000 # RESERVED 13
  longs   0x00000000 #pack <l SW_Software_IRQhandler
  longs   0x00000000 # RESERVED 15
  longs   0x00000000 #pack <l WWDG_IRQhandler
  longs   0x00000000 #pack <l PVD_IRQhandler
  longs   0x00000000 #pack <l FLASH_IRQhandler
  longs   0x00000000 #pack <l RCC_IRQhandler
  longs   0x00000000 #pack <l EXTI7_0_IRQhandler
  longs   0x00000000 #pack <l AWU_IRQhandler
pack <l DMA1_CH1_IRQhandler				# DMA CH1 interrupt vector
  longs   0x00000000 #pack <l DMA1_CH2_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH3_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH4_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH5_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH6_IRQhandler
  longs   0x00000000 #pack <l DMA1_CH7_IRQhandler
  longs   0x00000000 #pack <l ADC1_IRQhandler				
  longs   0x00000000 #pack <l I2C1_EV_IRQhandler
  longs   0x00000000 #pack <l I2C1_ER_IRQhandler
  longs   0x00000000 #pack <l USART1_IRQhandler
  longs   0x00000000 #pack <l SPI1_IRQhandler
  longs   0x00000000 #pack <l TIM1BRK_IRQhandler
  longs   0x00000000 #pack <l TIM1UP_IRQhandler
  longs   0x00000000 #pack <l TIM1TRG_COM_IRQhandler
  longs   0x00000000 #pack <l TIM1CC_IRQhandler
  longs   0x00000000 #pack <l TIM2_IRQhandler

reset_handler:


    	li sp, STACK			# load stack pointer with stack end address
	 
    	li t0, vtable			#BASEADDR[31:2],The interrupt vector table base address,which needs to be 1KB aligned
    	ori t0, t0, 3			#BASEADDR[31:2],1: Identify by absolute address,1: Address offset based on interrupt number *4
    	#csrrw zero,t0, mtvec		# write to mtvec
	longs 0x30529073  
    
   	li t0,main
	longs 0x34129073          	#csrw	mepc,t0 :mepc updated with address of main
	longs 0x30200073		# mret ( return from interrupt)	.
  
	align 4
main:
	nop
	li x10,result1			# clear result1 variable
	sw x0,0(x10)
	li x10,result2			# clear result1 variable
	sw x0,0(x10)


###########
#enable periphrel clocks
	li x10,R32_RCC_APB2PCENR	# load address of APB2PCENR register to x10 ,for enabling GPIO A,D,C peripherals
	lw x11,0(x10)			# load contents from peripheral register R32_RCC_APB2PCENR pointed by x10
	li x7,((1<<0)|(1<<2)|(1<<4)|(1<<5)|(1<<14)|(1<<9))	# 1<<IOPA_EN,1<<IOPC_EN,1<<IOPD_EN,1<<USART_EN
	or x11,x11,x7			# or values 
	sw x11,0(10)			# store modified enable values in R32_RCC_APB2PCENR

#enable AHB periphrel clocks
	li x10,R32_RCC_AHBPCENR		# load address of AHBPCENR register to x10 ,for enabling DMA peripheral
	lw x11,0(x10)			# load contents from peripheral register R32_RCC_AHBPCENR pointed by x10
	li x7,(1<<0)			# 1<<DMA1_EN
	or x11,x11,x7			# or values 
	sw x11,0(10)			# store modified enable values in R32_RCC_APB2PCENR

#configure GPIO 
	li x10,R32_GPIOD_CFGLR		# load pointer x10 with address of R32_GPIOD_CFGLR , GPIO configuration register
	lw x11,0(x10)			# load contents from register pointed by x10
	li x7,~((0xf<<20)|(0xf<<24)|(0xf<<16)|(0xf<<12))	#clear pd6,pd5,pd4,pd3. we need to setup PD5 & PD6 for usart tx and rx and pd4,pd3 for ADC
	and x11,x11,x7			# clear pd4,pd5,pd6 mode and cnf bits for selected pin D4,D5,D6
	li x7,((0x8<<24)|(0xB<<20))	# pd6 = input with PU/PD,pd5= multiplex pushpull output 50mhz,pd4,pd3= analoge input 0b0000 (AIN4,AIN7)
	or x11,x11,x7			# OR value to register
	sw x11,0(x10)			# store in R32_GPIOD_CFGLR

#enable pull up for input	
	li x10,R32_GPIOD_OUTDR		# enable pullup resistor by setting OUTDR register
	lw x11,0(x10)			# this setting of GPIO_OUTDR for pullup resistor effects if corresonding pin is selected as input
	li x7,(1<<6)			#when PD6 is input with resistor selected 1= pullup and 0 = pulldown
	or x11,x11,x7
	sw x11,0(x0)


#####################

#configure USART baud
	li x10,R32_USART_BRR		# USART BAUD setting
	lw x11,0(x10)			# copy R32_USART_BRR to x11
	li x7,((52<<4)|(1<<0))		# 52.1 in BRR =9600
	or x11,x11,x7			# or registers
	sw x11,0(x10)			# store in R32_USART_BRR

#setup UART control and enable	
	li x10,R32_USART_CTLR1		# load x10 with R32_USART_CTLR1 address
	lw x11,0(x10)			# load to x11 contents
	li x7,(1<<13)|(1<<3)|(1<<2)	# enable USART UE, TX,RX bits		# UE 
	or x11,x11,x7
	sw x11,0(x10)			# store back new values

#############
#disable ADC before configuration
	li x10,R32_ADC_CTLR2
	lw x11,0(x10)
	li x7,0xfffffffe		# disable ADON bit , 0 written in 0bit
	and x11,x11,x7
	sw x11,0(x10)

#set number of conversions
	li x10,R32_ADC_RSQR1		# sequence register, number of conversion in bit 20-23
	lw x11,0(x10)
	li x7,(1<<20)			# AIN4 & AIN7 2 channels need conversion ,0000=1 and 1111=16 , 2=1 
	or x11,x11,x7
	sw x11,0(x10)

#set sequence of conversion channel
	li x10,R32_ADC_RSQR3		# sequence register, if multiple sensors used we can determine the order of conversion
	lw x11,0(x10)
	li x7,((7<<0)|(4<<5))		# AIN4 adc channel is written to sequence 1 and AIN7 ti sequence 2 
	or x11,x11,x7
	sw x11,0(x10)

#set sampling cycles
	li x10,R32_ADC_SAMPTR2		# takes sampling cycles for each measurement
	lw x11,0(x10)
	li x7,(7<<12)|(7<<21)		# 0b111/0x7 is 241 cycles	
	or x11,x11,x7
	sw x11,0(x10)

#enable adc in control register2
	li x10,R32_ADC_CTLR2
	lw x11,0(x10)
	li x7,(1<<0)			# enable ADC and ADON
	or x11,x11,x7
	sw x11,0(x10)

#reset calibrate ADC
	li x10,R32_ADC_CTLR2
	lw x11,0(x10)
	li x7,(1<<3)			# reset calibration	
	or x11,x11,x7
	sw x11,0(x10)
check1:
	li x10,R32_ADC_CTLR2
	lw x11,0(x10)
	andi x11,x11,(1<<3)
	bnez x11,check1

# calibrate ADC
	li x10,R32_ADC_CTLR2
	lw x11,0(x10)
	li x7,(1<<2)			# start calibration	
	or x11,x11,x7
	sw x11,0(x10)
check2:
	li x10,R32_ADC_CTLR2
	lw x11,0(x10)
	andi x11,x11,(1<<2)
	bnez x11,check2

# enable DMA clock
	li x10,R32_RCC_AHBPCENR
	lw x11,0(x10)
	ori x11,x11,(1<<0)		# 1<<DMA1EN , enable DMA1 clock
	sw x11,0(10)

	li x10,R32_DMA_PADDR1		# pripheral register address (source)
	lw x11,0(x10)
	li x7,R32_ADC_RDATAR		# address of ADC data register
	or x11,x11,x7
	sw x11,0(x10)

	li x10,R32_DMA_MADDR1		# memrory destination address register
	lw x11,0(x10)
	li x7,result1			# result1 is in SRAM ,address of result1 loaded in x7
	or x11,x11,x7
	sw x11,0(x10)

	li x10,R32_DMA_CNTR1		# data size expected
	lw x11,0(x10)
	li x7,2				# 2 data ( 2 adc readings)
	or x11,x11,x7
	sw x11,0(x10)

#configure DMA
	li x10,R32_DMA_CFGR1
	lw x11,0(x10)
	li x7,((3<<12)|(2<<10)|(2<<8)|(1<<7)|(1<<5))	#0<<MEM2MEM,0<<PL(13,12),10<<Msize(12,10),01<<Psize(9,8),1<<MINC(7),0<<PINC(6),1<<CIRC(5),0<<DIR(4),0<<TEIE(3),0<<HTIE(2),1<<TCIE(1),1<<EN(0)
	or x11,x11,x7
	sw x11,0(x10)

	li x10,R32_DMA_CFGR1
	lw x11,0(x10)
	li x7,(1<<0)			# 1<<EN(0), enable DMA
	or x11,x11,x7
	sw x11,0(x10)

#enable scan mode in ADC contorl register1 as we have more than 1 channel
	li x10,R32_ADC_CTLR1
	lw x11,0(x10)
	li x7,(1<<8)			# enable scan bit 1<<SCAN
	or x11,x11,x7
	sw x11,0(x10)

#configure in ADC control register2 with DMA,continues,and alignment
	li x10,R32_ADC_CTLR2		# 
	lw x11,0(x10)
	li x7,((1<<8)|(1<<1)|(7<<17)|(1<<0))		# 1<<DMA,1<<CONT,7<<17 software trigger to start conversion, enable ADC
	or x11,x11,x7
	sw x11,0(x10)

#trigger start adc conversion
	li x10,R32_ADC_CTLR2
	lw x11,0(x10)
	li x7,(1<<22)			# start ADC conversion with  SWSTART bit
	or x11,x11,x7
	sw x11,0(x10)
####################


PFIC_CONFIG:
	li x10,R32_PFIC_CFGR		# reset core PFIC register for interrupts
	lw x11,0(x10)
	li x7,((PFIC_KEY3<<16)|(1<<7))	# key3  and SYSRESET , reference manual tells to do it
	or x11,x11,x7
	sw x11,0(x10)			# store back new values

	li x10,R32_PFIC_IENR1		# PFIC Interrupt Enable in core PFIC
	lw x11,0(x10)
	li x7,(1<<22)			# enabled DMA1 CH1 interrupts in PFIC
	or x11,x11,x7
	sw x11,0(x10)			# store back new values

# enabling GLOBAL INTERRUPTS
	li t0, 0x88			# load MPIE and MIE bits , 1<<MIE in mstatus is enabling GLOBAL INTERRUPTS
	longs 0x30029073        	#csrw	mstatus,t0 ,manually assembled opcade to do csrrw the values in t0,  




#main endless loop for uart transmit

example:

	li x10,name			# load address of label "name" to x10, string to be transmitted
string_loop:
	lb x8,0(x10)			# load 1 byte from 0 offset of "name"
	beqz x8,finish			# if byte in x8 null branch to label "finish"
	call USART_TX			# call subroutine USART_TX to transmit byte
	addi x10,x10,1			# increase pointer by 1 byte
	j string_loop			# jump back to label string_loop until null is encountered
finish:

	call delay			# some delay
	
	
	li x10,result1			# point location with x10 to perform D_ASCII subroutine
	call D_ASCII			# procedure to convert result to ASCII , subroutine called with x10 pointing to result1 in SRAM
	call print			# prints result in USART
	li x8,' '			# load space (0x20) , print space
	call USART_TX			# call uart
	li x8,0x0d			# line feed	
	call USART_TX			# call uart
	li x8,0x0a			# carriage feed
	call USART_TX			# call uart

	call delay			# some delay

	li x10,result2			# point location with x10 to perform D_ASCII subroutine
	call D_ASCII			# procedure to convert result to ASCII , subroutine called with x10 pointing to result1 in SRAM
	call print			# prints result in USART
	li x8,' '			# load space (0x20) , print space
	call USART_TX			# call uart
	li x8,0x0d			# line feed	
	call USART_TX			# call uart
	li x8,0x0a			# carriage feed
	call USART_TX			# call uart

blank_line:
	li x8,' '
	call USART_TX			# call uart
	li x8,0x0d			# line feed
	call USART_TX			# call uart
	li x8,0x0a			# carriage feed
	call USART_TX			# call uart

	call delay			# delay for man in front of terminal

		
	j example
#############################################################################################################
# SUBROUTINES
############################################################################################################	
USART_TX:
	addi sp,sp,-16			# add space in stack
	sw ra,0(sp)			# push ra
	sw x7,4(sp)			# push x7
	sw x10,8(sp)			# push x10
	sw x11,12(sp)			# push x11

	li x10,R32_USART_STATR		# load address of usart status register
	lw x11,0(x10)			# load contents of status register in x11
	andi x11,x11,(1<<7)		# mask out 7th bit, transmit buffer empty flag
	beqz x11,USART_TX		# if 0 transmit buffer full, wait until bit is set
	#li x8,0x30
	mv x7,x8			# move byte in x8 to x7
	li x10,R32_USART_DATAR		# x10 has the address of data register
	sb x7,0(x10)			#store byte in x7 to data register
TC_check:
	li x10,R32_USART_STATR		# get contents of status register again
	lw x11,0(x10)
	andi x11,x11,(1<<6)		# check transmit complete bit
	beqz x11,TC_check		# wait if bit is 0 , when transmit complete = 1
		
	lw x11,12(sp)			# pop x11
	lw x10,8(sp)			# pop x10
	lw x7,4(sp)			# pop x7
	lw ra,0(sp)			# pop ra
	addi sp,sp,16			# set SP back 16 bytes
	ret				# return to caller

###################################################
# DELAY
###################################################
delay:	
	addi sp,sp,-8			# move sp 2 words
	sw ra,0(sp)			# push ra
	sw x6,4(sp)			# push x6
	li x6,2000000			# load an arbitarary value 20000000 to t1 register		
dloop:
	addi x6,x6,-1			# subtract 1 from t1
	bne x6,zero,dloop		# if t1 not equal to 0 branch to label loop
	lw x6,4(sp)			# pop x6
	lw ra,0(sp)			# pop ra
	addi sp,sp,8			# sp back 2 words
	ret				# return to caller
###########################################################
name:
string ADC DMA EXAMPLE
eol:
bytes 0x0d,0x0a,0x00

##########################################################################################################
# converts 1 byte into ASCII represented hexadecimal value
##########################################################################################################
bin_to_ascii:
	addi sp,sp,-4
	sw ra,0(sp)
	mv a3,t2
	andi a3,a3,0xf0
	srli a3,a3,4
	slti a4,a3,10			# set a4 to 1 if a3 is less than 10 ,10and higher a4=0
	beqz a4 ,letter1
	ori a3,a3,0x30
	#mv a0,a3
	mv x8,a3
	call USART_TX
	j low_nibble
letter1:
	addi a3,a3,0x37
	#mv a0,a3
	mv x8,a3
	call USART_TX
low_nibble:
	mv a3,t2
	andi a3,a3,0x0f
	slti a4,a3,10			# set a4 to 1 if a3 is less than 10 ,10and higher a4=0
	beqz a4 ,letter2
	ori a3,a3,0x30
	#mv a0,a3
	mv x8,a3
	call USART_TX
	j exit_bin_to_ascii
letter2:
	addi a3,a3,0x37
	#mv a0,a3
	mv x8,a3
	call USART_TX
exit_bin_to_ascii:
	lw ra,0(sp)
	addi sp,sp,4
	ret
#######################################################################################################

#########################################################################################################################################
# D_ASCII subroutine for converting binary in result1 to DECIMAL (ASCII), point with x10 to register with value to be converted
#########################################################################################################################################

D_ASCII:			
	addi sp,sp,-32		# adjust stack pointer
	sw ra,28(sp)		# PUSH
	sw x15,24(sp)		# PUSH
	sw x11,20(sp)		# PUSH
	sw x8,16(sp)		# PUSH
	sw x7,12(sp)		# PUSH
	sw x5,8(sp)		# PUSH
	sw x4,4(sp)		# PUSH
	sw t1,0(sp)		# PUSH
	li x4,0			# clear register
	li x5,0			# clear register
	li x7,0			# clear register
	li x8,0			# clear register
	li x15,0		# clear register
	
#	li x10,0x20000010	# result1 (point address of value that has to be converted with x10)
	lw x4,0(x10)		# copy value from memory pointed by x10 to x4,this routine to be called after pointing to register with required value
#	li x4,0xffffffff	# 32bit word to be converted into ascii chars
	li x7,1000000000	# divisor
Y1:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X1		# if result negative(not divisible) branch to X1
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y1			# jump to label Y1 till not divisible
X1:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	li x10,mem		# set pointer x10 to SRAM register mem to store the byte
	sb x15,0(x10)		# store byte in mem+0
	li x15,0		# clear result
	li x7,100000000		# load x7 with new divisor
Y2:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X2		# if result negative(not divisible) branch to X2
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y2			# jump to label Y2 till not divisible
X2:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+1 to store the byte
	sb x15,0(x10)		# store byte in mem+1
	li x15,0		# clear result
	li x7,10000000		# load x7 with new divisor
Y3:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X3		# if result negative(not divisible) branch to X3
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y3			# jump to label Y3 till not divisible
X3:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+2 to store the byte
	sb x15,0(x10)		# store byte in mem+2
	li x15,0		# clear result
	li x7,1000000		# load x7 with new divisor
Y4:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X4		# if result negative(not divisible) branch to X4
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y4			# jump to label Y4 till not divisible
X4:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+3		
	sb x15,0(x10)		# store byte in mem+3
	li x15,0		# clear result
	li x7,100000		# load x7 with new divisor
Y5:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X5		# if result negative(not divisible) branch to X5
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y5			# jump to label Y5 till not divisible
X5:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+4 to store the byte
	sb x15,0(x10)		# store byte in mem+4
	li x15,0		# clear result
	li x7,10000		# load x7 with new divisor
Y6:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X6		# if result negative(not divisible) branch to X6
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y6			# jump to label Y6 till not divisible
X6:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+5 to store the byte
	sb x15,0(x10)		# store byte in mem+5
	li x15,0		# clear result
	li x7,1000		# load x7 with new divisor
Y7:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X7		# if result negative(not divisible) branch to X7
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y7			# jump to label Y7 till not divisible
X7:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+6 to store the byte
	sb x15,0(x10)		# store byte in mem+6
	li x15,0		# clear result
	li x7,100		# load x7 with new divisor
Y8:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X8		# if result negative(not divisible) branch to X8
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y8			# jump to label Y8 till not divisible
X8:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+7 to store the byte
	sb x15,0(x10)		# store byte in mem+7
	li x15,0		# clear result
	li x7,10		# load x7 with new divisor
Y9:
	sub x8,x4,x7		# subtract divisor from word to be converted
	sltu x5,x4,x7		# if result negative set x5 to indicate cannot be divided
	bnez x5,X9		# if result negative(not divisible) branch to X9
	mv x4,x8		# else move remainder to x4 for further division/subtraction
	addi x15,x15,1		# increase result by 1
	j Y9			# jump to label Y9 till not divisible
X9:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+8 to store the byte
	sb x15,0(x10)		# store byte in mem+8
	li x15,0		# clear result
	mv x15,x4
X10:
	addi x15,x15,0x30	# add ascii 0 to result to convert to ASCII char
	addi x10,x10,1		# set pointer x10 to SRAM register mem+9 to store the byte	
	sb x15,0(x10)		# store byte in mem+9

	lw t1,0(sp)		# POP
	lw x4,4(sp)		# POP
	lw x5,8(sp)		# POP
	lw x7,12(sp)		# POP
	lw x8,16(sp)		# POP
	lw x11,20(sp)		# POP
	lw x15,24(sp)		# POP
	lw ra,28(sp)		# POP
	addi sp,sp,32		# adjust stack pointer
	ret			# return to caller
######################################################

################################################################################
# Prints 10 bytes from result1 to USART with lead 0 suppress, values to be stored in mem0-mem9
################################################################################
print:
	addi sp,sp,-20		# adjust stack pointer
	sw ra,16(sp)		# PUSH
	sw x11,12(sp)		# PUSH
	sw x10,8(sp)		# PUSH
	sw x8,4(sp)		# PUSH
	sw t1,0(sp)		# PUSH

	li x11,scratch		# point x11 to scratch register in SRAM
	sw zero,0(x11)		# clear scratch register

	li x10,mem		# point to address mem ,top byte stored in mem0 lowest byte in mem+9, need to print top byte 1st
	li t1,0			# byte counter loaded with 10 , total 10 bytes to be printed
	li x15,10		# max count of 10bytes in x15

Z:
	lb x8,0(x10)		# load byte from result1, msb to lsb
	
	li x4,0x30		# load ascii 0 in x4
	beq x8,x4,supress0	# if result1 byte in x8 is equal to ascii 0 in x4 branch to suppress0 label
	li x11,scratch		# point x11 to scratch if x8 is not 0,that means we have found the 1st byte that is not 0, all leading 0s finished
	li x5,1			# load x5 with 1
	sw x5,0(x11)		# store in scratch register in sram , used as a flag to indicate all leading 0s are finished

print1:
	addi x10,x10,1		# increase the address by 1 byte
	addi t1,t1,1		# increase the byte counter once
	call USART_TX		# call uart
	bne t1,x15,Z		# if t1 not equal to 10 as loaded in x15 loop back to print1 till al 10 bytes are transmitted via usart
	lw t1,0(sp)		# POP
	lw x8,4(sp)		# POP
	lw x10,8(sp)		# POP
	lw x11,12(sp)		# POP
	lw ra,16(sp)		# POP
	addi sp,sp,20		# adjust stack pointer
	ret			# return to caller
supress0:
	li x11,scratch		# point x11 to scratch	
	lw x5,0(x11)		# copy value of scratch to x5
	bnez x5,no_more_supress	# if x5 is not 0 branch to label "no_more_supress"
	li x8,0x20		# if x5 is 0 load x8 with space/blank	
	li x4,9			# load x4 with value 9, suppose the whole value is 0, we dont want to display blank space, test this is 9th byte 2nd last byte
	beq t1,x4,last0is0	# if t1 equals 9 in x4 branch to label "last0is0" which will keep last 0 as 0 on screen
no_more_supress:
	J print1		# no 0 suppression jump to print1	
last0is0:
	li x8,0x30		# load ascii 0 for last 0
	J print1		# jump to print1

##########################################################
# DMA ISR
##########################################################

DMA1_CH1_IRQhandler:
	addi sp,sp,-60    		# adjust stack pointer
	sw x15,56(sp)			# PUSH
	sw x14,52(sp)			# PUSH
	sw x13,48(sp)			# PUSH
	sw x12,44(sp)			# PUSH
	sw x11,40(sp)			# PUSH
	sw x10,36(sp)			# PUSH
	sw x9,32(sp)			# PUSH
	sw x8,28(sp)			# PUSH
	sw x7,24(sp)			# PUSH
	sw x6,20(sp)			# PUSH
	sw x5,16(sp)			# PUSH
	sw x4,12(sp)			# PUSH
	sw x3,8(sp)			# PUSH
	sw x2,4(sp)			# PUSH
	sw x1,0(sp)			# PUSH
	
###########
	li x10,R32_DMA_INTFCR
	lw x11,0(x10)
	li x7,(1<<0)
	or x11,x11,x7
	sw x11,0(x10)

	

############
	lw x1,0(sp)			# POP
	lw x2,4(sp)			# POP
	lw x3,8(sp)			# POP
	lw x4,12(sp)			# POP
	lw x5,16(sp)			# POP
	lw x6,20(sp)			# POP
	lw x7,24(sp)			# POP
	lw x8,28(sp)			# POP
	lw x9,32(sp)			# POP
	lw x10,36(sp)			# POP
	lw x11,40(sp)			# POP
	lw x12,44(sp)			# POP
	lw x13,48(sp)			# POP
	lw x14,52(sp)			# POP
	lw x15,56(sp)			# POP
	addi sp,sp,60			# adjust stack pointer
	longs 0x30200073		# mret (manually assembled opcode for mret as per RISCV spec)
	
#################	