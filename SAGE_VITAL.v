// ============================================================
// SIH26181 — AI-Powered Personal Health Companion (Qualcomm Inc.)
// Regime-Adaptive Edge Watchdog + Wake Controller
//
// CHANGE FROM PREVIOUS VERSION:
// 'rst' is no longer an external port. There is no valid I/O
// Planner row for a user-driven internal reset on this board -
// nRST/nSLEEP are board-level pins wired to the MCU, not your
// design. Reset is now generated internally by a simple power-on
// counter (por_reset) that holds reset high for a fixed number of
// clock cycles after the oscillator starts, then releases it.
//
// clk is still a top-level port - assign it to OSC_CLK in the
// I/O Planner (Clock tab), NOT a GPIO pin.
//
// 4 FPGA input channels (8-bit each), 2 outputs:
//   risk_out   [7:0] - risk-relevant feature value
//   wake_flag  [1 bit] - HIGH only in Risk/Critical regime, tells
//                        the ESP32-S3 to wake and run its classifier
//
// The FPGA does NOT classify. It only computes a feature value and
// decides whether to wake the ML model. All classification (SVM/
// TinyML) runs on the ESP32-S3 only.
// ============================================================

// ---------- Module 0: Internal power-on reset generator ----------
module por_reset (
    input  wire clk,
    output reg  rst
);

    // holds rst high for 15 clock cycles after power-up, then releases
    reg [3:0] por_count;

    initial begin
        por_count = 4'd0;
        rst       = 1'b1;
    end

    always @(posedge clk) begin
        if (por_count != 4'd15) begin
            por_count <= por_count + 1'b1;
            rst       <= 1'b1;
        end else begin
            rst <= 1'b0;
        end
    end

endmodule


// ---------- Module 1: Serial receiver (4 bytes = 32 bits) ----------
module serial_rx (
    input  wire        clk,
    input  wire        rst,
    input  wire        serial_data,
    input  wire        serial_clk,
    output reg  [7:0]  stress_in,
    output reg  [7:0]  bpm_in,
    output reg  [7:0]  temp_in,
    output reg  [7:0]  gas_in,
    output reg         data_ready
);

    reg sclk_sync1, sclk_sync2, sclk_prev;
    reg data_sync1, data_sync2;
    reg [31:0] shift_reg;
    reg [5:0]  bit_count;

    wire sclk_rising = sclk_sync2 & ~sclk_prev;
    wire [31:0] shift_next = {shift_reg[30:0], data_sync2};

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sclk_sync1 <= 1'b0;
            sclk_sync2 <= 1'b0;
            sclk_prev  <= 1'b0;
            data_sync1 <= 1'b0;
            data_sync2 <= 1'b0;
            shift_reg  <= 32'd0;
            bit_count  <= 6'd0;
            stress_in  <= 8'd0;
            bpm_in     <= 8'd0;
            temp_in    <= 8'd0;
            gas_in     <= 8'd0;
            data_ready <= 1'b0;
        end else begin
            sclk_sync1 <= serial_clk;
            sclk_sync2 <= sclk_sync1;
            sclk_prev  <= sclk_sync2;
            data_sync1 <= serial_data;
            data_sync2 <= data_sync1;

            data_ready <= 1'b0;

            if (sclk_rising) begin
                shift_reg <= shift_next;

                // 32 bits = 4 bytes: stress, bpm, temp, gas (MSB first)
                if (bit_count == 6'd31) begin
                    stress_in  <= shift_next[31:24];
                    bpm_in     <= shift_next[23:16];
                    temp_in    <= shift_next[15:8];
                    gas_in     <= shift_next[7:0];
                    data_ready <= 1'b1;
                    bit_count  <= 6'd0;
                end else begin
                    bit_count <= bit_count + 1'b1;
                end
            end
        end
    end

endmodule


// ---------- Module 2: Three-regime watchdog + wake controller ----------
module regime_adaptive_health_core (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire [7:0]  stress_in,
    input  wire [7:0]  bpm_in,
    input  wire [7:0]  temp_in,
    input  wire [7:0]  gas_in,
    output reg  [7:0]  risk_out,
    output reg         wake_flag
);

    localparam IDLE      = 3'd0;
    localparam CHEAP     = 3'd1;
    localparam CHECK     = 3'd2;
    localparam RISK_CALC = 3'd3;
    localparam ESC_INIT  = 3'd4;
    localparam ESC_ITER  = 3'd5;
    localparam OUT_ST    = 3'd6;

    reg [2:0] state;

    // two thresholds define three regimes - RETUNE against real
    // sensor ranges before demo, these are placeholders
    localparam [7:0] THRESH_LOW  = 8'd60;
    localparam [7:0] THRESH_HIGH = 8'd120;

    reg [1:0] regime; // 0 = stable, 1 = risk, 2 = critical

    // cheap path: bpm successive difference fused with other 3 channels
    reg [7:0] bpm_prev;
    reg [7:0] cheap_sum;
    wire [7:0] diff_w      = (bpm_in >= bpm_prev) ? (bpm_in - bpm_prev) : (bpm_prev - bpm_in);
    wire [9:0] cheap_sum_w = diff_w + stress_in + temp_in + gas_in;

    // risk path: 4-sample bpm history, windowed average with other channels
    reg [7:0] bpm_hist0, bpm_hist1, bpm_hist2, bpm_hist3;
    reg [7:0] risk_result;
    wire [10:0] risk_sum_w = bpm_hist0 + bpm_hist1 + bpm_hist2 + bpm_hist3
                             + stress_in + temp_in + gas_in;

    // critical path: reduced cordic on bpm_in / stress_in pair
    reg signed [9:0] cx;
    reg signed [9:0] cy;
    reg [2:0] iter_count;
    localparam NUM_ITERS = 4;
    reg [7:0] escalate_result;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state           <= IDLE;
            regime          <= 2'd0;
            bpm_prev        <= 8'd0;
            cheap_sum       <= 8'd0;
            bpm_hist0       <= 8'd0;
            bpm_hist1       <= 8'd0;
            bpm_hist2       <= 8'd0;
            bpm_hist3       <= 8'd0;
            risk_result     <= 8'd0;
            cx              <= 10'd0;
            cy              <= 10'd0;
            iter_count      <= 3'd0;
            escalate_result <= 8'd0;
            risk_out        <= 8'd0;
            wake_flag       <= 1'b0;
        end else begin
            case (state)

                IDLE: begin
                    if (start)
                        state <= CHEAP;
                end

                CHEAP: begin
                    cheap_sum <= cheap_sum_w[9:2]; // divide by 4

                    bpm_hist3 <= bpm_hist2;
                    bpm_hist2 <= bpm_hist1;
                    bpm_hist1 <= bpm_hist0;
                    bpm_hist0 <= bpm_in;

                    bpm_prev <= bpm_in;
                    state    <= CHECK;
                end

                CHECK: begin
                    if (cheap_sum <= THRESH_LOW) begin
                        regime <= 2'd0;
                        state  <= OUT_ST;
                    end else if (cheap_sum <= THRESH_HIGH) begin
                        regime <= 2'd1;
                        state  <= RISK_CALC;
                    end else begin
                        regime <= 2'd2;
                        state  <= ESC_INIT;
                    end
                end

                RISK_CALC: begin
                    risk_result <= risk_sum_w[10:3]; // divide by 8
                    state       <= OUT_ST;
                end

                ESC_INIT: begin
                    cx         <= {2'b00, bpm_in};
                    cy         <= {2'b00, stress_in};
                    iter_count <= 3'd0;
                    state      <= ESC_ITER;
                end

                ESC_ITER: begin
                    if (iter_count < NUM_ITERS) begin
                        if (cy[9] == 1'b0) begin
                            cx <= cx + (cy >>> iter_count);
                            cy <= cy - (cx >>> iter_count);
                        end else begin
                            cx <= cx - (cy >>> iter_count);
                            cy <= cy + (cx >>> iter_count);
                        end
                        iter_count <= iter_count + 1'b1;
                    end else begin
                        if (cx[9] == 1'b0)
                            escalate_result <= cx[8:1];
                        else
                            escalate_result <= 8'd255;

                        state <= OUT_ST;
                    end
                end

                OUT_ST: begin
                    case (regime)
                        2'd0: begin
                            risk_out  <= cheap_sum;
                            wake_flag <= 1'b0;
                        end
                        2'd1: begin
                            risk_out  <= risk_result;
                            wake_flag <= 1'b1;
                        end
                        2'd2: begin
                            risk_out  <= escalate_result;
                            wake_flag <= 1'b1;
                        end
                        default: begin
                            risk_out  <= cheap_sum;
                            wake_flag <= 1'b0;
                        end
                    endcase
                    state <= IDLE;
                end

                default: begin
                    state <= IDLE;
                end

            endcase
        end
    end

endmodule


// ---------- Module 3: Top-level wrapper ----------
// NOTE: 'rst' is NOT a port anymore. 'clk' must be assigned to
// OSC_CLK in the I/O Planner's Clock tab, not a GPIO pin.
module sentinel_vitals_top (
    input  wire clk,
    input  wire serial_data,
    input  wire serial_clk,
    output wire [7:0] risk_out,
    output wire       wake_flag
);

    wire rst_int;

    por_reset u_por (
        .clk(clk),
        .rst(rst_int)
    );

    wire [7:0] stress_in;
    wire [7:0] bpm_in;
    wire [7:0] temp_in;
    wire [7:0] gas_in;
    wire       data_ready;

    serial_rx u_rx (
        .clk(clk),
        .rst(rst_int),
        .serial_data(serial_data),
        .serial_clk(serial_clk),
        .stress_in(stress_in),
        .bpm_in(bpm_in),
        .temp_in(temp_in),
        .gas_in(gas_in),
        .data_ready(data_ready)
    );

    regime_adaptive_health_core u_core (
        .clk(clk),
        .rst(rst_int),
        .start(data_ready),
        .stress_in(stress_in),
        .bpm_in(bpm_in),
        .temp_in(temp_in),
        .gas_in(gas_in),
        .risk_out(risk_out),
        .wake_flag(wake_flag)
    );

endmodule
