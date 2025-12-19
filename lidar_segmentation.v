// ==========================================
// 1. BUFFER (Registru de Deplasare)
// ==========================================
module window_buffer #(parameter DATA_WIDTH = 16)(
    input wire clk,
    input wire rst_n,
    input wire valid_in,
    input wire [DATA_WIDTH-1:0] data_in,
    output reg [DATA_WIDTH-1:0] p0, p1, p2, p3, p4,
    output reg valid_out
);
    reg [2:0] counter;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p0 <= 0; p1 <= 0; p2 <= 0; p3 <= 0; p4 <= 0;
            valid_out <= 0; counter <= 0;
        end else if (valid_in) begin
            p4 <= p3; p3 <= p2; p2 <= p1; p1 <= p0; p0 <= data_in;
            if (counter < 5) counter <= counter + 1;
            else valid_out <= 1;
        end
    end
endmodule

// ==========================================
// 2. FILTRU SAVITZKY-GOLAY (Netezire)
// ==========================================
module savitzky_golay_filter #(parameter DATA_WIDTH = 16)(
    input wire clk, rst_n, valid_in,
    input wire signed [DATA_WIDTH-1:0] p0, p1, p2, p3, p4,
    output reg signed [DATA_WIDTH-1:0] filtered_val,
    output reg valid_out
);
    reg signed [DATA_WIDTH+5:0] sum;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum <= 0; filtered_val <= 0; valid_out <= 0;
        end else if (valid_in) begin
            sum <= (17 * p2) + (12 * p3) + (12 * p1) - (3 * p4) - (3 * p0);
            filtered_val <= sum >>> 5; 
            valid_out <= 1;
        end else valid_out <= 0;
    end
endmodule

// ==========================================
// 3. LOGICA DE SEGMENTARE (CORECȚIA ESTE AICI)
// ==========================================
// AM SCHIMBAT SLOPE_THRESHOLD DIN 50 IN 8
module segmentation_logic #(parameter DATA_WIDTH = 16, parameter SLOPE_THRESHOLD = 8)(
    input wire clk, rst_n, valid_in,
    input wire signed [DATA_WIDTH-1:0] current_z,
    input wire [DATA_WIDTH-1:0] current_r,
    output reg is_ground,
    output reg valid_out
);
    reg signed [DATA_WIDTH-1:0] prev_z;
    reg [DATA_WIDTH-1:0] prev_r;
    reg [DATA_WIDTH-1:0] delta_z, delta_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_z <= 0; prev_r <= 0; is_ground <= 0; valid_out <= 0;
        end else if (valid_in) begin
            prev_z <= current_z;
            prev_r <= current_r;
            
            // Calcul diferență absolută
            delta_z = (current_z > prev_z) ? (current_z - prev_z) : (prev_z - current_z);
            delta_r = (current_r > prev_r) ? (current_r - prev_r) : (prev_r - current_r);
            
            // Verificare pantă cu noul prag (8)
            // Daca diferenta de inaltime e mica fata de distanta, e sol.
            if (delta_z < (delta_r * SLOPE_THRESHOLD)) 
                is_ground <= 1'b1; 
            else 
                is_ground <= 1'b0;
            
            valid_out <= 1'b1;
        end else valid_out <= 0;
    end
endmodule

// ==========================================
// 4. TOP MODULE
// ==========================================
module lidar_ground_segmentation_top #(parameter DATA_WIDTH = 16)(
    input wire clk,
    input wire rst_n,
    input wire data_valid_in,
    input wire [DATA_WIDTH-1:0] raw_z_in,
    input wire [DATA_WIDTH-1:0] raw_r_in,
    output wire segmentation_result,
    output wire result_valid
);
    wire [DATA_WIDTH-1:0] w_p0, w_p1, w_p2, w_p3, w_p4;
    wire buffer_valid, filter_valid;
    wire [DATA_WIDTH-1:0] filtered_z;
    
    window_buffer #(.DATA_WIDTH(DATA_WIDTH)) u_buffer (
        .clk(clk), .rst_n(rst_n), .valid_in(data_valid_in), .data_in(raw_z_in),
        .p0(w_p0), .p1(w_p1), .p2(w_p2), .p3(w_p3), .p4(w_p4),
        .valid_out(buffer_valid)
    );
    
    savitzky_golay_filter #(.DATA_WIDTH(DATA_WIDTH)) u_filter (
        .clk(clk), .rst_n(rst_n), .valid_in(buffer_valid),
        .p0(w_p0), .p1(w_p1), .p2(w_p2), .p3(w_p3), .p4(w_p4),
        .filtered_val(filtered_z),
        .valid_out(filter_valid)
    );
    
    // Observa ca nu mai transmitem parametrul aici, deci el va lua valoarea default (8) din modul
    segmentation_logic #(.DATA_WIDTH(DATA_WIDTH)) u_seg (
        .clk(clk), .rst_n(rst_n), .valid_in(filter_valid),
        .current_z(filtered_z), .current_r(raw_r_in), 
        .is_ground(segmentation_result), .valid_out(result_valid)
    );
endmodule