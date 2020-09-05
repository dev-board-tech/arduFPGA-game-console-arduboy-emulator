/*
 * This IP is the MEGA/XMEGA TOP implementation.
 * 
 * Copyright (C) 2020  Iulian Gheorghiu (morgoth@devboard.tech)
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
 
`timescale 1ns / 1ps

`include "mega-def.v"

// For REV1.0 short PIN36 and PIN 35 beween them, BTN_BACK has been moved from PIN35 to PIN36.
`define REV							"1.1"


`define USE_PIO_B					"TRUE"
`define USE_PIO_C					"TRUE"
`define USE_PIO_D					"TRUE"
`define USE_PIO_E					"TRUE"
`define USE_PIO_F					"TRUE"
`define USE_PLL						"TRUE"
`define USE_PLL_HI_FREQ				"FALSE"
`define USE_TIMER_0					"TRUE"
`define USE_TIMER_1					"FALSE"
`define USE_TIMER_3					"TRUE"
`define USE_TIMER_4					"TRUE"
`define USE_SPI_1					"TRUE"
`define USE_UART_1					"TRUE"
`define USE_TWI_1					"FALSE"
`define USE_EEPROM					"TRUE"
`define USE_RNG_AS_ADC				"TRUE"
`define USE_COMPOSITE_VIDEO_OUT		"TRUE"

`define PLATFORM					"iCE40UP"
`define FLASH_ROM_FILE_NAME			"l1_boot_ld"
 
module top(
	output RGB0, 
	output RGB1, 
	output RGB2, 
	output BUZ_L,
	output BUZ_G,
	output BUZ_R,
	output OLED_DC,
	output OLED_SS,
	output OLED_RST,
	output SCK,
	output MOSI,
	input MISO,
	input BTN_RIGHT,
	input BTN_LEFT,
	input BTN_UP,
	input BTN_DN,
	input BTN_BACK,
	input BTN_OK,
	input BTN_INTERRUPT,
	output DES_SS,
	output uSD_SS,
	input uSD_CD,
	output APP_SS,
	output VS_RST,
	output VS_xCS,
	output VS_xDCS,
	input VS_DREQ,
	output UART_TX,
	inout UART_RX
	);

wire pll_locked;
reg [3:0]pll_locked_buf;
wire sys_clk;
wire sys_rst = ~pll_locked_buf[3];
wire pll_clk = sys_clk;
wire sys_clk_int;
reg [1:0]sys_clk_t;


always @ (posedge sys_clk)
begin
	pll_locked_buf <= {pll_locked_buf[2:0], pll_locked}; 
end

HSOSC
#(
  .CLKHF_DIV ("0b00")
) HSOSC_inst (
  .CLKHFPU (1'b1),  // I
  .CLKHFEN (1'b1),  // I
  .CLKHF   (clk)   // O
);
//synthesis ROUTE_THROUGH_FABRIC = 0;
/* synthesis ROUTE_THROUGH_FABRIC= [0|1] */

PLL_DEV_48M PLL_inst(
	.ref_clk_i(clk),
	.bypass_i(1'b0),
	.rst_n_i(1'b1), 
	.lock_o(pll_locked), 
	.outcore_o(sys_clk_int), 
	.outglobal_o(sys_clk) 
);


//always @ (posedge sys_clk_int) sys_clk_t <= sys_clk_t + 2'h1;
 
wire [7:0]io_addr;
wire [7:0]io_out;
wire io_write;
wire [7:0]io_in;
wire io_read;
wire io_rst;

wire ssd1306_scl;
wire ssd1306_dc;
wire ssd1306_ss;
wire ssd1306_rst;
wire vs_rst;
wire [2:0]ld;

wire nmi_sig;
wire nmi_ack;
wire nmi_rst;

wire sec_reg_rst;
wire sec_en;

wire buz_r, buz_l;
wire [1:0]volume;

wire disc_usr_kbd;
wire enable_usb_ntsc_out;
wire enable_aud_ntsc_out;

reg BTN_INTERRUPT_reg, BTN_BACK_reg, BTN_OK_reg, BTN_UP_reg, BTN_DN_reg, BTN_LEFT_reg, BTN_RIGHT_reg, uSD_CD_reg;
reg BTN_BACK_reg_sys, BTN_OK_reg_sys, BTN_UP_reg_sys, BTN_DN_reg_sys, BTN_LEFT_reg_sys, BTN_RIGHT_reg_sys;
always @ (*)
begin
	if(disc_usr_kbd)
	begin
		BTN_BACK_reg <= 1'b1;
		BTN_OK_reg <= 1'b1;
		BTN_UP_reg <= 1'b1;
		BTN_DN_reg <= 1'b1;
		BTN_LEFT_reg <= 1'b1;
		BTN_RIGHT_reg <= 1'b1;
	end
	else
	begin
		BTN_BACK_reg <= BTN_BACK;
		BTN_OK_reg <= BTN_OK;
		BTN_UP_reg <= BTN_UP;
		BTN_DN_reg <= BTN_DN;
		BTN_LEFT_reg <= BTN_LEFT;
		BTN_RIGHT_reg <= BTN_RIGHT;
	end

	BTN_BACK_reg_sys <= BTN_BACK;
	BTN_OK_reg_sys <= BTN_OK;
	BTN_UP_reg_sys <= BTN_UP;
	BTN_DN_reg_sys <= BTN_DN;
	BTN_LEFT_reg_sys <= BTN_LEFT;
	BTN_RIGHT_reg_sys <= BTN_RIGHT;
end

always @ *
begin
	BTN_INTERRUPT_reg = BTN_INTERRUPT;
	uSD_CD_reg = uSD_CD;
end

wire UART_TX_UC;
wire UART_RX_UC;

atmega32u4_arduboy # (
	.PLATFORM(`PLATFORM),
	.BOOT_ADDR(16'h7800),
	.ARDU_FPGA_ICE40UP5K_GAME("TRUE"),
	
	.CORE_TYPE(`MEGA_ENHANCED_128K),
	.ROM_ADDR_WIDTH(15), // 14 = 16K Words / 32K Bytes; 15 = 32K Words / 64K Bytes; 16 = 64K Words / 128K Bytes Not supported yet.
	.BOOT_ADDR_WIDTH(11), // 1024 Words / 2048 Bytes, how big the first stage boot-loader ROM to be.
	.BUS_ADDR_DATA_LEN(16), // Max 64K Bytes.
	.RAM_TYPE("SRAM"),  // "BLOCK","SRAM"// If "SRAM" is choosen, will be a 32KB block of RAM.
	.RAM_ADDR_WIDTH(15), // 32KB, if you use "SRAM" this value need to be 15.
	.EEP_ADDR_WIDTH(10), // 1K Bytes.
	.RESERVED_RAM_FOR_IO(12'h100), // Lowest 256 Bytes of RAM addresses are reserved for IO's.
	.VECTOR_INT_TABLE_SIZE(43),// 42 of original ATmega32U4 + NMI
	.WATCHDOG_CNT_WIDTH(0),//27 // We do not use watchdog, is not a critical design and most of arduboy games does not use him.


	.REGS_REGISTERED("FALSE"),
	.ROM_PATH(`FLASH_ROM_FILE_NAME),
	.USE_PIO_B(`USE_PIO_B),
	.USE_PIO_C(`USE_PIO_C),
	.USE_PIO_D(`USE_PIO_D),
	.USE_PIO_E(`USE_PIO_E),
	.USE_PIO_F(`USE_PIO_F),
	.USE_PLL(`USE_PLL),
	.USE_PLL_HI_FREQ(`USE_PLL_HI_FREQ),
	.USE_TIMER_0(`USE_TIMER_0),
	.USE_TIMER_1(`USE_TIMER_1),
	.USE_TIMER_3(`USE_TIMER_3),
	.USE_TIMER_4(`USE_TIMER_4),
	.USE_SPI_1(`USE_SPI_1),
	.USE_UART_1(`USE_UART_1),
	.USE_TWI_1(`USE_TWI_1),
	.USE_EEPROM(`USE_EEPROM),
	.USE_RNG_AS_ADC(`USE_RNG_AS_ADC)
) atmega32u4_arduboy_inst (
	.core_rst(sys_rst),
	.dev_rst(sys_rst),
	.clk(sys_clk),
	.clk_pll(pll_clk),
	.nmi_sig(nmi_sig),
	.nmi_ack(nmi_ack),
	.sec_reg_rst(sec_reg_rst),
	.sec_en(sec_en),
    .buttons({BTN_BACK_reg, BTN_OK_reg, BTN_UP_reg, BTN_DN_reg, BTN_LEFT_reg, BTN_RIGHT_reg}),
    .RGB(ld),
    .Buzzer1(buz_l),
    .Buzzer2(buz_r),
    .OledDC(ssd1306_dc),
    .OledCS(ssd1306_ss),
    .OledRST(ssd1306_rst),
    .spi_scl(ssd1306_scl),
    .spi_mosi(MOSI),
	.spi_miso(MISO),
	.uSD_CS(uSD_SS),
	.uSD_CD(uSD_CD_reg),
	.ADC_CS(ADC_SS),
	.VS_RST(vs_rst),
	.VS_xCS(VS_xCS),
	.VS_xDCS(VS_xDCS),
	.VS_DREQ(VS_DREQ),
	.uart_tx(UART_TX_UC),
	.uart_rx(UART_RX_UC),
	.twi_scl(),
	.twi_sda(),

	.io_addr(io_addr),
	.io_out(io_out),
	.io_write(io_write),
	.io_in(io_in),
	.io_read(io_read),
	.io_sel(io_sel),
	.io_rst(io_rst),
	.nmi_rst(nmi_rst)
);
 
rtc #(
	.PERIOD_STATIC(16000),
	.CNT_SIZE(14)
	)rtc_inst(
	.rst_i(nmi_rst),
	.clk_i(sys_clk),
	.int_o(nmi_sig),
	.int_ack_i(nmi_ack)
	);
 
wire [0:0]dummy_out_port_a;
wire [7:0]dat_pa_d_out;
atmega_pio # (
	.PLATFORM(`PLATFORM),
	.BUS_ADDR_DATA_LEN(8),
	.PORT_WIDTH(8),
	.USE_CLEAR_SET("FALSE"),
	.PORT_OUT_ADDR('h22),
	.DDR_ADDR('h21),
	.PIN_ADDR('h20),
	.PORT_CLEAR_ADDR('h01),	
	.PORT_SET_ADDR('h02),
	.PINMASK(			  8'b11111111),
	.PULLUP_MASK(		  8'b00000000),
	.PULLDN_MASK(		  8'b00000000),
	.INVERSE_MASK(		  8'b00000000),
	.OUT_ENABLED_MASK(	  8'b01111111),
	.INITIAL_OUTPUT_VALUE(8'b00000011),
	.INITIAL_DIR_VALUE(   8'b01111111)
)pio_a(
	.rst_i(sys_rst),
	.clk_i(sys_clk),
	.addr_i(io_addr[7:0]),
	.wr_i(io_write),
	.rd_i(io_read),
	.bus_i(io_out),
	.bus_o(dat_pa_d_out),

	.io_i({BTN_UP_reg_sys, BTN_DN_reg_sys, BTN_BACK_reg_sys, BTN_OK_reg_sys, BTN_INTERRUPT_reg, BTN_LEFT_reg_sys, BTN_RIGHT_reg_sys, 1'b0}),
	.io_o({dummy_out_port_a, enable_aud_ntsc_out, enable_usb_ntsc_out, disc_usr_kbd, volume, APP_SS, DES_SS}),
	.pio_out_io_connect_o()
	);


wire [1:0]ntsc_out;

generate
if( `USE_COMPOSITE_VIDEO_OUT == "TRUE")
begin
	
wire ntsc_clk = clk;
wire [12:0]lcd_h_cnt;
wire [12:0]lcd_v_cnt;

wire pixel_is_visible;
wire [0:0]ssd1306_rgb_data;

assign UART_TX = enable_usb_ntsc_out ? ntsc_out[0] : UART_TX_UC;
assign UART_RX = enable_usb_ntsc_out ? ntsc_out[1] : 1'bz;
assign UART_RX_UC = enable_usb_ntsc_out ? 1'b1 : UART_RX;

localparam [3:0]  SIGNAL_LEVEL_SYNC         = 4'b0000,
                    SIGNAL_LEVEL_BLANK        = 4'b0001,
                    SIGNAL_LEVEL_DARK_GREY    = 4'b0011,
                    SIGNAL_LEVEL_LIGHT_GREY   = 4'b0111,
                    SIGNAL_LEVEL_WHITE        = 4'b1111;

interlaced_ntsc # (
	.PIXEL_NUANCE_DEPTH(1)
)interlaced_ntsc_inst(
    .rst_i((~enable_usb_ntsc_out & ~enable_aud_ntsc_out) | sys_rst),
    .clk_i(ntsc_clk),
    .pixel_data_i(pixel_is_visible ? (ssd1306_rgb_data[0] ? SIGNAL_LEVEL_WHITE : SIGNAL_LEVEL_BLANK) : SIGNAL_LEVEL_BLANK),
    .h_sync_out_o(), // single clock tick indicating pixel_y will incrememt on next clock (for debugging)
    .v_sync_out_o(), // single clock tick indicating pixel_y will reset to 0 or 1 on next clock, depending on the field (for debugging)
    .pixel_y_o(lcd_v_cnt[9:0]),    // which line
    .pixel_x_o(lcd_h_cnt[9:0]),
    .pixel_is_visible_o(pixel_is_visible),
    .ntsc_out_o(ntsc_out)
);

ssd1306 # (
	.X_OLED_SIZE(128),
	.Y_OLED_SIZE(64),
	.X_PARENT_SIZE(560),
	.Y_PARENT_SIZE(400),
	.PIXEL_INACTIVE_COLOR(1'b0),
	.PIXEL_ACTIVE_COLOR(1'b1),
	.INACTIVE_DISPLAY_COLOR(32'h10101010),
	.VRAM_BUFFERED_OUTPUT("TRUE"),
	.FULL_COLOR_OUTPUT("FALSE")
)ssd1306_inst(
	.rst_i(~ssd1306_rst),
	.clk_i(sys_clk),
	
	.edge_color_i(1'b0),
	.raster_x_i(lcd_h_cnt),
	.raster_y_i(lcd_v_cnt),
	.raster_clk_i(ntsc_clk),
	.raster_d_o(ssd1306_rgb_data),
	
	.ss_i(ssd1306_ss),
	.scl_i(ssd1306_scl),
	.mosi_i(MOSI),
	.dc_i(ssd1306_dc)
);

end
endgenerate

wire buzr, buzl;
pwm # (
	.WIDTH(4)
) pwm_l (
	.rst_i(io_rst),
	.clk_i(sys_clk),
	.en_i(buz_l),
	.hiz_i(vs_rst),
	.val_i({2'h0, ~volume}),
	.pwm_o(buzl)
);

pwm # (
	.WIDTH(4)
) pwm_r (
	.rst_i(io_rst),
	.clk_i(sys_clk),
	.en_i(buz_r),
	.hiz_i(vs_rst),
	.val_i({2'h0, ~volume}),
	.pwm_o(buzr)
);


assign BUZ_G = (vs_rst & ~enable_aud_ntsc_out) ? 1'bz : 1'b0;
assign BUZ_L = enable_aud_ntsc_out ? ntsc_out[0] : buzl;
assign BUZ_R = enable_aud_ntsc_out ? ntsc_out[1] : buzr;

assign io_in = dat_pa_d_out;

assign {OLED_DC, OLED_SS, OLED_RST, SCK} = {ssd1306_dc, ssd1306_ss, ssd1306_rst, ssd1306_scl};

assign VS_RST = vs_rst;


BB_OD LED_B_Inst (
  .T_N (1'b1),  // I
  .I   (~ld[2]),  // I
  .O   (),  // O
  .B   (RGB2)   // IO
);
BB_OD LED_G_Inst (
  .T_N (1'b1),  // I
  .I   (~ld[1]),  // I
  .O   (),  // O
  .B   (RGB1)   // IO
);
BB_OD LED_R_Inst (
  .T_N (1'b1),  // I
  .I   (~ld[0]),  // I
  .O   (),  // O
  .B   (RGB0)   // IO
);
endmodule