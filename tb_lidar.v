`timescale 1ns / 1ps

module tb_lidar;

    // ==========================================
    // 1. DEFINIRE SEMNALE ȘI PARAMETRI
    // ==========================================
    parameter DATA_WIDTH = 16;
    
    // Intrări pentru modulul testat (reg)
    reg clk;
    reg rst_n;
    reg data_valid_in;
    reg signed [DATA_WIDTH-1:0] raw_z_in; // 'signed' pentru că filtrul lucrează cu numere cu semn
    reg [DATA_WIDTH-1:0] raw_r_in;
    
    // Ieșiri de la modulul testat (wire)
    wire segmentation_result; // 1 = Sol, 0 = Obstacol
    wire result_valid;
    
    // ==========================================
    // 2. INSTANȚIEREA MODULULUI PRINCIPAL (UUT)
    // ==========================================
    lidar_ground_segmentation_top #(.DATA_WIDTH(DATA_WIDTH)) uut (
        .clk(clk),
        .rst_n(rst_n),
        .data_valid_in(data_valid_in),
        .raw_z_in(raw_z_in),
        .raw_r_in(raw_r_in),
        .segmentation_result(segmentation_result),
        .result_valid(result_valid)
    );
    
    // ==========================================
    // 3. GENERAREA CEASULUI
    // ==========================================
    // Perioada = 10ns (Frecvența = 100 MHz)
    always #5 clk = ~clk;
    
    // ==========================================
    // 4. SCENARIUL DE TEST (STIMULI)
    // ==========================================
    initial begin
        // --- CONFIGURARE PENTRU GTKWAVE ---
        // Aceste linii sunt esențiale pentru a vedea graficele!
        $dumpfile("rezultate.vcd"); 
        $dumpvars(0, tb_lidar);
        
        // --- INIȚIALIZARE ---
        clk = 0;
        rst_n = 0;
        data_valid_in = 0;
        raw_z_in = 0;
        raw_r_in = 0;
        
        $display("--- Start Simulare ---");
        
        // --- RESET ---
        #20 rst_n = 1; // Eliberăm resetul după 20ns
        #10;
        
        // ============================================================
        // TEST CAZ 1: TEREN PLAT (SOL)
        // Înălțimea Z rămâne mică/constantă, Raza R crește.
        // Așteptare: segmentation_result trebuie să fie 1.
        // ============================================================
        $display("Timp: %0t -> Testare SOL PLAT...", $time);
        
        repeat(15) begin
            @(posedge clk);        // Sincronizare cu ceasul
            data_valid_in = 1;
            raw_z_in = 16'd10;     // Înălțime constantă (ex: 10mm)
            raw_r_in = raw_r_in + 10; // Ne îndepărtăm de senzor
        end
        
        // ============================================================
        // TEST CAZ 2: OBSTACOL BRUSC (ZID/BORDURĂ)
        // Înălțimea Z sare brusc la o valoare mare.
        // Așteptare: segmentation_result trebuie să devină 0.
        // ============================================================
        $display("Timp: %0t -> Testare OBSTACOL...", $time);
        
        repeat(10) begin
            @(posedge clk);
            data_valid_in = 1;
            raw_z_in = 16'd200;    // Săritură la 200mm (obstacol)
            raw_r_in = raw_r_in + 10;
        end
        
        // ============================================================
        // TEST CAZ 3: REVENIRE PE SOL
        // Înălțimea Z revine la valoarea mică.
        // Așteptare: segmentation_result revine la 1.
        // ============================================================
        $display("Timp: %0t -> Revenire la SOL...", $time);
        
        repeat(15) begin
            @(posedge clk);
            data_valid_in = 1;
            raw_z_in = 16'd10;     // Din nou jos
            raw_r_in = raw_r_in + 10;
        end

        // Oprim datele valide
        @(posedge clk);
        data_valid_in = 0;
        
        // Mai lăsăm simularea să curgă puțin pentru a vedea ultimele rezultate
        #50;
        
        $display("--- Final Simulare ---");
        $finish; // Oprește simularea
    end
    
endmodule