`include "clockworks.v"


module Memory (
   input             clk,
   input      [31:0] mem_raddr, // address to be read from
   output reg [31:0] mem_rdata, // data read from memory
   input             mem_rstrb, // goes high when processor wants to read
   input      [31:0] mem_waddr, // address to be written to
   input      [31:0] mem_wdata,
   input      [3:0]  mem_wmask
);
    reg [31:0] MEM [0:255]; 

    `ifdef BENCH
    localparam slow_bit=1;
    `else
    localparam slow_bit=21;
    `endif

    `include "riscv_assembly.v"
    integer L0_   = 8;
    integer wait_ = 32;
    integer L1_   = 40;

    initial begin
        LI(s0,0);
        LI(s1,20);
    Label(L0_); 
        LB(a0,s0,400); // LEDs are plugged on a0 (=x10)
        CALL(LabelRef(wait_));
        ADDI(s0,s0,1); 
        BNE(s0,s1, LabelRef(L0_));
        EBREAK();

    Label(wait_);
        LI(t0,1);
        SLLI(t0,t0,slow_bit);
    Label(L1_);
        ADDI(t0,t0,-1);
        BNEZ(t0,LabelRef(L1_));
        RET();

        endASM();

        // Note: index 100 (word address) corresponds
        // to address 400 (byte address)
        MEM[100] = {
            8'b0000_0100,
            8'b0000_0010,
            8'b0000_0001,
            8'b0000_0000
        };
        MEM[101] = {
            8'b0001_0010,
            8'b0001_0001,
            8'b0001_0000,
            8'b0000_1000
        };
        MEM[102] = {
            8'b0001_1010,
            8'b0001_1001,
            8'b0001_1000,
            8'b0001_0100
        };
        MEM[103] = {
            8'b0001_1111,
            8'b0001_1110,
            8'b0001_1101,
            8'b0001_1100
        };
        MEM[104] = {
            8'b0000_0001,
            8'b0000_0011,
            8'b0000_0111,
            8'b0000_1111
        };
    end
    always @(posedge clk) begin
        if (mem_rstrb) begin
            mem_rdata <= MEM[mem_raddr[31:2]];
        end

        if (mem_wmask[0])
            MEM[mem_waddr[31:2]][ 7:0 ] <= mem_wdata[ 7:0 ];
        if (mem_wmask[1])
            MEM[mem_waddr[31:2]][15:8 ] <= mem_wdata[15:8 ];
        if (mem_wmask[2])
            MEM[mem_waddr[31:2]][23:16] <= mem_wdata[23:16];
        if (mem_wmask[3])
            MEM[mem_waddr[31:2]][31:24] <= mem_wdata[31:24];

    end
endmodule

module Processor (
    input             clk,
    input             resetn,
    output     [31:0] mem_raddr,
    input      [31:0] mem_rdata,
    output            mem_rstrb,
    output     [31:0] mem_waddr,
    output     [31:0] mem_wdata,
    output      [3:0] mem_wmask,
    output reg [31:0] x10
);
    reg [31:0] PC = 0;
    reg [31:0] instr;

    wire isALUreg  =  (instr[6:0] == 7'b0110011); // rd <- rs1 OP rs2   
    wire isALUimm  =  (instr[6:0] == 7'b0010011); // rd <- rs1 OP Iimm
    wire isBranch  =  (instr[6:0] == 7'b1100011); // if (rs1 OP rs2) PC <- PC+Bimm
    wire isJALR    =  (instr[6:0] == 7'b1100111); // rd <- PC+4; PC <- rs1+Iimm
    wire isJAL     =  (instr[6:0] == 7'b1101111); // rd <- PC+4; PC <- PC+Jimm
    wire isAUIPC   =  (instr[6:0] == 7'b0010111); // rd <- PC + Uimm
    wire isLUI     =  (instr[6:0] == 7'b0110111); // rd <- Uimm   
    wire isLoad    =  (instr[6:0] == 7'b0000011); // rd <- mem[rs1+Iimm]
    wire isStore   =  (instr[6:0] == 7'b0100011); // mem[rs1+Simm] <- rs2
    wire isSYSTEM  =  (instr[6:0] == 7'b1110011); // special

    wire [4:0] rs1Id = instr[19:15];
    wire [4:0] rs2Id = instr[24:20];
    wire [4:0] rdId  = instr[11:7];

    wire [2:0] funct3 = instr[14:12];
    wire [6:0] funct7 = instr[31:25];

    wire [31:0] Uimm={    instr[31],   instr[30:12], {12{1'b0}}};
    wire [31:0] Iimm={{21{instr[31]}}, instr[30:20]};
    wire [31:0] Simm={{21{instr[31]}}, instr[30:25],instr[11:7]};
    wire [31:0] Bimm={{20{instr[31]}}, instr[7],instr[30:25],instr[11:8],1'b0};
    wire [31:0] Jimm={{12{instr[31]}}, instr[19:12],instr[20],instr[30:21],1'b0};

    reg [31:0] RegisterBank [0:31];
    `ifdef BENCH
        integer i;
        initial begin
            for(i=0; i<32; ++i) begin
                RegisterBank[i] = 0;
            end
        end
    `endif

    reg [31:0] rs1 = 0;
    reg [31:0] rs2 = 0;

    wire [31:0] load_addr = rs1 + Iimm;
    wire [31:0] LOAD_word = mem_rdata;
    wire [15:0] LOAD_halfword = load_addr[1] ? LOAD_word[31:16] : LOAD_word[15:0];
    wire [7:0] LOAD_byte = load_addr[0] ? LOAD_halfword[15:8] : LOAD_halfword[7:0];
    reg[31:0] LOAD_value;
    reg LOAD_sign;
    reg[31:0] loadOut;
    always @(*) begin
        case(funct3)
            3'b000: begin
                LOAD_value = LOAD_byte;
                LOAD_sign = 0;
                loadOut = LOAD_value;
            end
            3'b001: begin
                LOAD_value = LOAD_halfword;
                LOAD_sign = 0;
                loadOut = LOAD_value;
            end
            3'b010: begin
                LOAD_value = LOAD_word;
                LOAD_sign = 0;
                loadOut = LOAD_value;
            end
            3'b100: begin
                LOAD_value = LOAD_byte;
                LOAD_sign = LOAD_byte[7];
                loadOut = { {24{LOAD_sign}}, LOAD_value};
            end
            3'b101: begin
                LOAD_value = LOAD_halfword;
                LOAD_sign = LOAD_halfword[15];
                loadOut = { {16{LOAD_sign}}, LOAD_value};
            end
            default: begin
                LOAD_value = 0;
                LOAD_sign = 0;
                loadOut = LOAD_value;
            end
        endcase
    end

    wire [31:0] store_addr = rs1 + Simm;
    assign mem_wdata[ 7: 0] = rs2[7:0];
    assign mem_wdata[15: 8] = store_addr[0] ? rs2[7:0]  : rs2[15: 8];
    assign mem_wdata[23:16] = store_addr[1] ? rs2[7:0]  : rs2[23:16];
    assign mem_wdata[31:24] = store_addr[0] ? rs2[7:0]  :
                              store_addr[1] ? rs2[15:8] : rs2[31:24];

    wire mem_byteAccess     = funct3[1:0] == 2'b00;
    wire mem_halfwordAccess = funct3[1:0] == 2'b01;
    wire [3:0] STORE_wmask =
        mem_byteAccess      ?
            (store_addr[1] ?
            (store_addr[0] ? 4'b1000 : 4'b0100) :
            (store_addr[0] ? 4'b0010 : 4'b0001)
                ) :
        mem_halfwordAccess ?
            (store_addr[1] ? 4'b1100 : 4'b0011) :
            4'b1111;

    wire [31:0] aluIn1 = rs1;
    wire [31:0] aluIn2 = isALUreg | isBranch ? rs2: Iimm;

    wire [31:0] aluPlus = aluIn1 + aluIn2;
    // Use a single 33 bits subtract to do subtraction and all comparisons
    wire [32:0] aluMinus = {1'b1, ~aluIn2} + {1'b0,aluIn1} + 33'b1;
    wire LT  = (aluIn1[31] ^ aluIn2[31]) ? aluIn1[31] : aluMinus[32];
    wire LTU = aluMinus[32];
    wire EQ  = (aluMinus == 0);

    wire [31:0] rightShifter = 
        $signed({funct7[5] & aluIn1[31], aluIn1}) >>> aluIn2[4:0];
    reg[31:0] aluOut;
    always @(*) begin
        case(funct3)
            3'b000: aluOut = (funct7[5] & instr[5]) ? aluMinus[31:0] : aluPlus;
            3'b001: aluOut = aluIn1 << aluIn2[4:0];
            3'b010: aluOut = {31'b0, LT};
            3'b011: aluOut = {31'b0, LTU};
            3'b100: aluOut = aluIn1 ^ aluIn2;
            3'b101: aluOut = rightShifter;
            3'b110: aluOut = aluIn1 | aluIn2;
            3'b111: aluOut = aluIn1 & aluIn2;
        endcase
    end

    reg takeBranch;
    always @(*) begin
        case (funct3)
            3'b000: takeBranch = EQ;
            3'b001: takeBranch = !EQ;
            3'b100: takeBranch = LT;
            3'b101: takeBranch = !LT;
            3'b110: takeBranch = LTU;
            3'b111: takeBranch = !LTU;
            default: takeBranch = 1'b0;
        endcase
    end

    // State machine
    localparam FETCH_INSTR = 0;
    localparam WAIT_INSTR  = 1;
    localparam FETCH_REGS  = 2;
    localparam EXECUTE     = 3;
    localparam LOAD        = 4;
    localparam WAIT_DATA   = 5;
    localparam STORE       = 6;
    reg [2:0] state = FETCH_INSTR;
    reg [31:0] writeBackData = 0;
    reg writeBackEn = 0;
    always @(posedge clk) begin
        if (!resetn) begin
            PC <= 0;
            instr <= 32'b0000000_00000_00000_000_00000_0110011; // NOP
            state <= FETCH_INSTR;
        end else begin
            writeBackEn = 0;
            case (state)
                FETCH_INSTR: begin
                    state <= WAIT_INSTR;
                end
                WAIT_INSTR: begin
                    instr <= mem_rdata;
                    state <= FETCH_REGS;
                end
                FETCH_REGS: begin
                    rs1 = RegisterBank[rs1Id];
                    rs2 = RegisterBank[rs2Id];
                    state <= EXECUTE;
                end
                EXECUTE: begin
                    PC <=
                        isSYSTEM ? PC :
                        (isBranch && takeBranch) ? PC+Bimm :
                        isJAL ? PC + Jimm : 
                        isJALR ? rs1 + Iimm : 
                        PC + 4;
                    writeBackEn = isALUreg || isALUimm || isJAL || isJALR || isLUI || isAUIPC;
                    writeBackData =
                        (isJAL || isJALR) ? (PC + 4) :
                        (isLUI) ? Uimm :
                        (isAUIPC) ? (PC + Uimm) :
                        aluOut;
                    state <= isLoad ? LOAD : isStore ? STORE : FETCH_INSTR;
                end
                LOAD: begin
                    state <= WAIT_DATA;
                end
                WAIT_DATA: begin
                    writeBackEn = 1'b1;
                    writeBackData = loadOut;
                    state <= FETCH_INSTR;
                end
                STORE: begin
                    state <= FETCH_INSTR;
                end
            endcase
            if (writeBackEn && rdId != 0) begin
                RegisterBank[rdId] <= writeBackData;
                if (rdId == 10) begin
                    x10 <= writeBackData;
                end
            end
        end
    end

    assign mem_raddr = (state == WAIT_INSTR || state == FETCH_INSTR) ? PC : load_addr;
    assign mem_rstrb = (state == FETCH_INSTR || state == LOAD);
    assign mem_waddr = store_addr;
    assign mem_wmask = {4{(state == STORE)}} & STORE_wmask;

    // Debug info
    `ifdef BENCH
        always @(posedge clk) begin
            if (state == FETCH_REGS) begin
                $display("PC=%0d, INSTR=%b_%b_%b", PC, instr[31:12], instr[11:7], instr[6:0]);
                case (1'b1)
                    isALUreg:
                        $display("ALUreg x%0d <- x%0d(%0d) x%0d(%0d) funct3=%b", rdId, rs1Id, rs1, rs2Id, rs2, funct3);
                    isALUimm:
                        $display("ALUimm x%0d <- x%0d(%0d) imm=%0d funct3=%b", rdId, rs1Id, rs1, Iimm, funct3);
                    isBranch:
                        $display("BRANCH");
                    isJAL:
                        $display("JAL");
                    isJALR:
                        $display("JALR");
                    isAUIPC:
                        $display("AUIPC");
                    isLUI:
                        $display("LUI");	
                    isLoad:
                        $display("LOAD");
                    isStore:
                        $display("STORE");
                    isSYSTEM:
                        $display("SYSTEM");
                    default begin $display("ERROR"); $finish(); end
                endcase
                if (isSYSTEM)
                    $finish();
            end
            if (state == EXECUTE) begin
                if (writeBackEn && rdId != 0) begin
                    $display("writeBackData=%0d", writeBackData);
                end
                $display("---");
            end
        end
    `endif
endmodule

module SOC (
    input  CLK,        // system clock
    input  RESET,      // reset button
    output [4:0] LEDS, // system LEDs
    input  RXD,        // UART receive
    output TXD         // UART transmit
);
    // Clock gearbox (to let you see what happens)
    // and reset circuitry (to workaround an
    // initialization problem with Ice40)
    Clockworks CW(
        .CLK(CLK),
        .RESET(RESET),
        .clk(clk),
        .resetn(resetn)
    );
    wire clk;    // internal clock
    wire resetn; // internal reset signal, goes low on reset

    Memory RAM(
        .clk(clk),
        .mem_raddr(mem_raddr),
        .mem_rdata(mem_rdata),
        .mem_rstrb(mem_rstrb),
        .mem_waddr(mem_waddr),
        .mem_wdata(mem_wdata),
        .mem_wmask(mem_wmask)
   );
    wire [31:0] mem_raddr;
    wire [31:0] mem_rdata;
    wire mem_rstrb;
    wire [31:0] mem_waddr;
    wire [31:0] mem_wdata;
    wire [3:0] mem_wmask;

    Processor CPU(
      .clk(clk),
      .resetn(resetn),
      .mem_raddr(mem_raddr),
      .mem_rdata(mem_rdata),
      .mem_rstrb(mem_rstrb),
      .mem_waddr(mem_waddr),
      .mem_wdata(mem_wdata),
      .mem_wmask(mem_wmask),
      .x10(x10)
   );
    wire [31:0] x10;

    // Plug the leds on register 1 to see its contents
    assign LEDS = x10[4:0];
    assign TXD  = 1'b0; // not used for now
endmodule
