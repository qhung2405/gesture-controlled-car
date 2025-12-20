`timescale 1ns/1ps
// ================================================================
//  Single-file nRF24L01+ PRX (Receiver) for Arty Z7 (Vivado 2018.2)
//
//  This version is aligned to the provided Arduino transmitter.ino:
//   - setChannel(76)
//   - setDataRate(RF24_1MBPS)
//   - setCRCLength(RF24_CRC_16)
//   - setPayloadSize(32)
//   - setPALevel(RF24_PA_LOW)
//   - setAutoAck(false)
//   - openWritingPipe("1Node")  (5 bytes: '1','N','o','d','e')
//
//  Debug: led[3:0] shows payload[0][3:0] (with your ino => should be 4'b0001)
// ================================================================


// -----------------------------
// SPI byte master (mode 0)
// - External CSN control
// - Shifts MSB first
// - Samples MISO on rising edge
// -----------------------------
module spi_byte_master #(
    parameter integer CLK_HZ = 125_000_000,
    parameter integer SCK_HZ = 5_000_000
)(
    input  wire       clk,
    input  wire       reset,      // active high

    input  wire       start,      // 1-cycle pulse
    input  wire [7:0] tx_byte,
    output reg  [7:0] rx_byte,
    output reg        busy,
    output reg        done,       // 1-cycle pulse

    output reg        sck,
    output reg        mosi,
    input  wire       miso
);
    // sck ~= CLK_HZ/(2*HALF_DIV)
    localparam integer HALF_DIV = (CLK_HZ/(2*SCK_HZ) < 1) ? 1 : (CLK_HZ/(2*SCK_HZ));

    reg [15:0] div_cnt;
    reg [7:0]  sh_tx;
    reg [7:0]  sh_rx;
    reg [2:0]  bit_cnt;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            div_cnt <= 0;
            sh_tx   <= 0;
            sh_rx   <= 0;
            bit_cnt <= 0;
            sck     <= 1'b0;
            mosi    <= 1'b0;
            rx_byte <= 8'h00;
            busy    <= 1'b0;
            done    <= 1'b0;
        end else begin
            done <= 1'b0;

            if (!busy) begin
                sck <= 1'b0;
                if (start) begin
                    busy    <= 1'b1;
                    div_cnt <= 0;
                    sh_tx   <= tx_byte;
                    sh_rx   <= 8'h00;
                    bit_cnt <= 3'd7;
                    mosi    <= tx_byte[7];  // present MSB before first rising edge
                end
            end else begin
                if (div_cnt == HALF_DIV-1) begin
                    div_cnt <= 0;
                    sck <= ~sck;

                    if (sck == 1'b0) begin
                        // rising edge: sample MISO
                        sh_rx <= {sh_rx[6:0], miso};
                    end else begin
                        // falling edge: update MOSI or finish
                        if (bit_cnt != 0) begin
                            bit_cnt <= bit_cnt - 1'b1;
                            mosi    <= sh_tx[bit_cnt-1];
                        end else begin
                            rx_byte <= sh_rx;
                            busy    <= 1'b0;
                            done    <= 1'b1;
                            sck     <= 1'b0;
                        end
                    end
                end else begin
                    div_cnt <= div_cnt + 1'b1;
                end
            end
        end
    end
endmodule


// ------------------------------------------------
// Top + nRF24 PRX FSM
// ------------------------------------------------
module top_artyz7_nrf24_rx_singlefile #(
    parameter integer CLK_HZ = 125_000_000,

    // --- nRF settings (MUST match transmitter.ino) ---
    parameter [7:0]   RF_CH      = 8'd76,
    parameter integer PAYLOAD_W  = 32,                     // fixed payload size
    // "1Node" => bytes: 31 4E 6F 64 65 (LSB-first over SPI write)
    parameter [39:0]  RX_ADDR_P0 = 40'h65646F4E31,

    parameter integer SPI_HZ     = 5_000_000
)(
    input  wire clk_125mhz,
    input  wire reset_btn,     // active high

    output wire nrf_mosi,
    input  wire nrf_miso,
    output wire nrf_sck,
    output reg  nrf_csn,
    output reg  nrf_ce,

    output reg  [3:0] led,
    
    output wire       motor_in1,
    output wire       motor_in2,
    output wire       motor_in3,
    output wire       motor_in4,
    output wire       motor_pwm_a,   
    output wire       motor_pwm_b
);

    // -------------------------
    // SPI byte engine
    // -------------------------
    reg        spi_start;
    reg [7:0]  spi_tx;
    wire [7:0] spi_rx;
    wire       spi_busy;
    wire       spi_done;

    spi_byte_master #(
        .CLK_HZ(CLK_HZ),
        .SCK_HZ(SPI_HZ)
    ) u_spi (
        .clk    (clk_125mhz),
        .reset  (reset_btn),
        .start  (spi_start),
        .tx_byte(spi_tx),
        .rx_byte(spi_rx),
        .busy   (spi_busy),
        .done   (spi_done),
        .sck    (nrf_sck),
        .mosi   (nrf_mosi),
        .miso   (nrf_miso)
    );

    // -------------------------
    // nRF commands & registers
    // -------------------------
    localparam [7:0] CMD_R_RX_PAYLOAD = 8'h61;
    localparam [7:0] CMD_FLUSH_RX     = 8'hE2;
    localparam [7:0] CMD_NOP          = 8'hFF;
    localparam [7:0] CMD_W_REGISTER   = 8'h20;

    localparam [7:0] REG_CONFIG       = 8'h00;
    localparam [7:0] REG_EN_AA        = 8'h01;
    localparam [7:0] REG_EN_RXADDR    = 8'h02;
    localparam [7:0] REG_SETUP_AW     = 8'h03;
    localparam [7:0] REG_RF_CH        = 8'h05;
    localparam [7:0] REG_RF_SETUP     = 8'h06;
    localparam [7:0] REG_STATUS       = 8'h07;
    localparam [7:0] REG_RX_ADDR_P0   = 8'h0A;
    localparam [7:0] REG_RX_PW_P0     = 8'h11;

    // -------------------------
    // Frame sequencer (CSN low, send N bytes, collect rx bytes)
    // -------------------------
    reg        frame_start;
    reg [5:0]  frame_len;
    reg [5:0]  frame_idx;
    reg [5:0]  frame_id;
    reg        frame_active;
    reg        frame_done_pulse;
    reg [7:0]  frame_rx_mem [0:40];

    // frame IDs
    localparam [5:0] F_W_EN_AA       = 6'd1;
    localparam [5:0] F_W_EN_RXADDR   = 6'd2;
    localparam [5:0] F_W_SETUP_AW    = 6'd3;
    localparam [5:0] F_W_RF_CH       = 6'd4;
    localparam [5:0] F_W_RF_SETUP    = 6'd5;
    localparam [5:0] F_W_RX_ADDR0    = 6'd6;
    localparam [5:0] F_W_RX_PW0      = 6'd7;
    localparam [5:0] F_W_CONFIG      = 6'd8;
    localparam [5:0] F_FLUSH_RX      = 6'd9;
    localparam [5:0] F_CLEAR_STATUS  = 6'd10;
    localparam [5:0] F_READ_STATUS   = 6'd11;
    localparam [5:0] F_READ_PAYLOAD  = 6'd12;

    // Lookup TX byte per frame_id + index
    function [7:0] tx_lookup;
        input [5:0] fid;
        input [5:0] idx;
        begin
            tx_lookup = CMD_NOP;
            case (fid)
                F_W_EN_AA: begin
                    if (idx==0) tx_lookup = CMD_W_REGISTER | REG_EN_AA;
                    else        tx_lookup = 8'h00; // transmitter uses setAutoAck(false)
                end
                F_W_EN_RXADDR: begin
                    if (idx==0) tx_lookup = CMD_W_REGISTER | REG_EN_RXADDR;
                    else        tx_lookup = 8'h01; // enable pipe0
                end
                F_W_SETUP_AW: begin
                    if (idx==0) tx_lookup = CMD_W_REGISTER | REG_SETUP_AW;
                    else        tx_lookup = 8'h03; // 5-byte address
                end
                F_W_RF_CH: begin
                    if (idx==0) tx_lookup = CMD_W_REGISTER | REG_RF_CH;
                    else        tx_lookup = RF_CH;
                end
                F_W_RF_SETUP: begin
                    // transmitter: setPALevel(RF24_PA_LOW) + setDataRate(1Mbps)
                    // => RF_PWR=01, RF_DR_LOW=0, RF_DR_HIGH=0, LNA_HCURR=1 => 0x03
                    if (idx==0) tx_lookup = CMD_W_REGISTER | REG_RF_SETUP;
                    else        tx_lookup = 8'h03;
                end
                F_W_RX_ADDR0: begin
                    // Write RX_ADDR_P0 (5 bytes), LSB first over SPI
                    if (idx==0)      tx_lookup = CMD_W_REGISTER | REG_RX_ADDR_P0;
                    else if (idx==1) tx_lookup = RX_ADDR_P0[7:0];
                    else if (idx==2) tx_lookup = RX_ADDR_P0[15:8];
                    else if (idx==3) tx_lookup = RX_ADDR_P0[23:16];
                    else if (idx==4) tx_lookup = RX_ADDR_P0[31:24];
                    else             tx_lookup = RX_ADDR_P0[39:32];
                end
                F_W_RX_PW0: begin
                    if (idx==0) tx_lookup = CMD_W_REGISTER | REG_RX_PW_P0;
                    else        tx_lookup = PAYLOAD_W[7:0];
                end
                F_W_CONFIG: begin
                    // PRX + PWR_UP + CRC16
                    if (idx==0) tx_lookup = CMD_W_REGISTER | REG_CONFIG;
                    else        tx_lookup = 8'h0F;
                end
                F_FLUSH_RX: begin
                    tx_lookup = CMD_FLUSH_RX;
                end
                F_CLEAR_STATUS: begin
                    if (idx==0) tx_lookup = CMD_W_REGISTER | REG_STATUS;
                    else        tx_lookup = 8'h70; // clear RX_DR/TX_DS/MAX_RT
                end
                F_READ_STATUS: begin
                    tx_lookup = CMD_NOP; // response byte = STATUS
                end
                F_READ_PAYLOAD: begin
                    if (idx==0) tx_lookup = CMD_R_RX_PAYLOAD;
                    else        tx_lookup = CMD_NOP;
                end
                default: tx_lookup = CMD_NOP;
            endcase
        end
    endfunction

    // Frame runner
    always @(posedge clk_125mhz or posedge reset_btn) begin
        if (reset_btn) begin
            frame_active     <= 1'b0;
            frame_done_pulse <= 1'b0;
            frame_idx        <= 0;
            spi_start        <= 1'b0;
            spi_tx           <= 8'h00;
            nrf_csn          <= 1'b1;
        end else begin
            frame_done_pulse <= 1'b0;
            spi_start        <= 1'b0;

            if (frame_start && !frame_active) begin
                frame_active <= 1'b1;
                frame_idx    <= 0;
                nrf_csn      <= 1'b0;
                spi_tx       <= tx_lookup(frame_id, 0);
                spi_start    <= 1'b1;
            end else if (frame_active) begin
                if (spi_done) begin
                    frame_rx_mem[frame_idx] <= spi_rx;

                    if (frame_idx == frame_len - 1) begin
                        frame_active     <= 1'b0;
                        frame_done_pulse <= 1'b1;
                        nrf_csn          <= 1'b1;
                    end else begin
                        frame_idx <= frame_idx + 1'b1;
                        spi_tx    <= tx_lookup(frame_id, frame_idx + 1'b1);
                        spi_start <= 1'b1;
                    end
                end
            end
        end
    end

    // -------------------------
    // Main FSM
    // -------------------------
    localparam [7:0] ST_RESET         = 8'd0;
    localparam [7:0] ST_STARTUP_DELAY = 8'd1;
    localparam [7:0] ST_DO_FRAME      = 8'd2;
    localparam [7:0] ST_WAIT_FRAME    = 8'd3;
    localparam [7:0] ST_PWRUP_DELAY   = 8'd4;
    localparam [7:0] ST_RUN           = 8'd5;

    reg [7:0] state;
    reg [7:0] state_after_frame;

    // delays
    localparam integer STARTUP_MS  = 5;
    localparam integer PWRUP_MS    = 3;
    localparam integer STARTUP_CYC = (CLK_HZ/1000)*STARTUP_MS;
    localparam integer PWRUP_CYC   = (CLK_HZ/1000)*PWRUP_MS;

    reg [31:0] delay_cnt;

    // poll interval
    localparam integer POLL_MS  = 1;
    localparam integer POLL_CYC = (CLK_HZ/1000)*POLL_MS;
    reg [31:0] poll_cnt;

    // status + payload storage
    reg [7:0] status_byte;
    reg [7:0] payload [0:31];
    integer i;

    task automatic kick_frame;
        input [5:0] fid;
        input [5:0] flen;
        input [7:0] next_st;
        begin
            frame_id          <= fid;
            frame_len         <= flen;
            frame_start       <= 1'b1;
            state             <= ST_WAIT_FRAME;
            state_after_frame <= next_st;
        end
    endtask

    always @(posedge clk_125mhz or posedge reset_btn) begin
        if (reset_btn) begin
            state       <= ST_RESET;
            frame_start <= 1'b0;
            nrf_ce      <= 1'b0;
            led         <= 4'b0000;

            delay_cnt   <= 0;
            poll_cnt    <= 0;
            status_byte <= 8'h00;

            for (i=0; i<32; i=i+1) payload[i] <= 8'h00;
        end else begin
            frame_start <= 1'b0;

            case (state)
                ST_RESET: begin
                    nrf_ce    <= 1'b0;
                    delay_cnt <= 0;
                    poll_cnt  <= 0;
                    state     <= ST_STARTUP_DELAY;
                end

                ST_STARTUP_DELAY: begin
                    if (delay_cnt >= STARTUP_CYC) begin
                        delay_cnt <= 0;
                        // init sequence like RF24 config (with our PRX tweaks)
                        kick_frame(F_W_EN_AA,      6'd2, ST_DO_FRAME);
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                ST_DO_FRAME: begin
                    // chain init frames based on the last completed frame_id
                    case (frame_id)
                        F_W_EN_AA:     kick_frame(F_W_EN_RXADDR,  6'd2, ST_DO_FRAME);
                        F_W_EN_RXADDR: kick_frame(F_W_SETUP_AW,   6'd2, ST_DO_FRAME);
                        F_W_SETUP_AW:  kick_frame(F_W_RF_CH,      6'd2, ST_DO_FRAME);
                        F_W_RF_CH:     kick_frame(F_W_RF_SETUP,   6'd2, ST_DO_FRAME);
                        F_W_RF_SETUP:  kick_frame(F_W_RX_ADDR0,   6'd6, ST_DO_FRAME);
                        F_W_RX_ADDR0:  kick_frame(F_W_RX_PW0,     6'd2, ST_DO_FRAME);
                        F_W_RX_PW0:    kick_frame(F_W_CONFIG,     6'd2, ST_DO_FRAME);
                        F_W_CONFIG:    kick_frame(F_FLUSH_RX,     6'd1, ST_DO_FRAME);
                        F_FLUSH_RX:    kick_frame(F_CLEAR_STATUS, 6'd2, ST_PWRUP_DELAY);
                        default:       state <= ST_PWRUP_DELAY;
                    endcase
                end

                ST_WAIT_FRAME: begin
                    if (frame_done_pulse) begin
                        case (frame_id)
                            F_READ_STATUS: begin
                                status_byte <= frame_rx_mem[0];
                                if (frame_rx_mem[0][6]) begin
                                    // RX_DR set => read payload (1 cmd + PAYLOAD_W bytes)
                                    kick_frame(F_READ_PAYLOAD, (6'd1 + PAYLOAD_W[5:0]), ST_WAIT_FRAME);
                                end else begin
                                    state <= ST_RUN;
                                end
                            end

                            F_READ_PAYLOAD: begin
                                for (i=0; i<32; i=i+1) begin
                                    if (i < PAYLOAD_W) payload[i] <= frame_rx_mem[i+1];
                                end
                                led <= frame_rx_mem[1][3:0]; // payload[0] low nibble
                                // clear IRQ flags then go back to run
                                kick_frame(F_CLEAR_STATUS, 6'd2, ST_RUN);
                            end

                            default: begin
                                state <= state_after_frame;
                            end
                        endcase
                    end
                end

                ST_PWRUP_DELAY: begin
                    if (delay_cnt >= PWRUP_CYC) begin
                        delay_cnt <= 0;
                        nrf_ce    <= 1'b1;   // start listening
                        poll_cnt  <= 0;
                        state     <= ST_RUN;
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                ST_RUN: begin
                    if (poll_cnt >= POLL_CYC) begin
                        poll_cnt <= 0;
                        kick_frame(F_READ_STATUS, 6'd1, ST_RUN);
                    end else begin
                        poll_cnt <= poll_cnt + 1'b1;
                    end
                end

                default: state <= ST_RESET;
            endcase
        end
    end
        Motor_Controller u_motor (
        .clk        (clk_125mhz),
        .reset      (reset_btn),
        .cmd_nibble (led),
        .motor_in1  (motor_in1),
        .motor_in2  (motor_in2),
        .motor_in3  (motor_in3),
        .motor_in4  (motor_in4),
        .motor_pwm_a(motor_pwm_a),
        .motor_pwm_b(motor_pwm_b)
    );
endmodule
