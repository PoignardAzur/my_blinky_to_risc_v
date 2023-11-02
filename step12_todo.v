`include "clockworks.v"

module Memory (
   input             clk,
   input      [31:0] mem_addr,  // address to be read
   output reg [31:0] mem_rdata, // data read from memory
   input             mem_rstrb  // goes high when processor wants to read
);
    reg [31:0] MEM [0:255]; 

    `include "riscv_assembly.v"
    integer L0_= 8;
    initial begin
        ADD(x1,x0,x0);
        ADDI(x2,x0,32);
    Label(L0_);
        ADDI(x1,x1,1);
        BNE(x1, x2, LabelRef(L0_));
        EBREAK();
    endASM();
    end

    always @(posedge clk) begin
        if (mem_rstrb) begin
            mem_rdata <= MEM[mem_addr[31:2]];
        end
    end
endmodule

module Processor (
    input             clk,
    input             resetn,
    output     [31:0] mem_addr,
    input      [31:0] mem_rdata,
    output            mem_rstrb,
    output reg [31:0] x1
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

    wire [31:0] aluIn1 = rs1;
    wire [31:0] aluIn2 = isALUreg ? rs2: Iimm;
    wire[4:0] shiftAmount = isALUreg ? rs2[4:0] : instr[24:20];
    reg[31:0] aluOut;
    always @(*) begin
        case(funct3)
            3'b000: aluOut = (funct7[5] & instr[5]) ? (aluIn1-aluIn2) : (aluIn1+aluIn2);
            3'b001: aluOut = aluIn1 << shiftAmount;
            3'b010: aluOut = ($signed(aluIn1) < $signed(aluIn2));
            3'b011: aluOut = aluIn1 < aluIn2;
            3'b100: aluOut = aluIn1 ^ aluIn2;
            3'b101: aluOut = funct7[5]? ($signed(aluIn1) >>> shiftAmount) : (aluIn1 >> shiftAmount); 
            3'b110: aluOut = aluIn1 | aluIn2;
            3'b111: aluOut = aluIn1 & aluIn2;
        endcase
    end

    wire [31:0] writeBackData =
        (isJAL || isJALR) ? (PC + 4) :
        (isLUI) ? Uimm :
        (isAUIPC) ? (PC + Uimm) :
        aluOut;
    wire instrHasWriteBack = isALUreg || isALUimm || isJAL || isJALR ||  isLUI || isAUIPC;

    reg takeBranch;
    always @(*) begin
        case (funct3)
            3'b000: takeBranch = rs1 == rs2;
            3'b001: takeBranch = rs1 != rs2;
            3'b100: takeBranch = ($signed(rs1) < $signed(rs2));
            3'b101: takeBranch = ($signed(rs1) >= $signed(rs2));
            3'b110: takeBranch = rs1 < rs2;
            3'b111: takeBranch = rs1 >= rs2;
            default: takeBranch = 1'b0;
        endcase
    end

    // State machine
    localparam FETCH_INSTR = 0;
    localparam WAIT_INSTR  = 1;
    localparam FETCH_REGS  = 2;
    localparam EXECUTE     = 3;
    reg [1:0] state = FETCH_INSTR;
    always @(posedge clk) begin
        if (!resetn) begin
            PC <= 0;
            instr <= 32'b0000000_00000_00000_000_00000_0110011; // NOP
            state <= FETCH_INSTR;
        end else begin
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
                    if (instrHasWriteBack && rdId != 0) begin
                        RegisterBank[rdId] <= writeBackData;
                        if (rdId == 1) begin
                            x1 <= writeBackData;
                        end
                    end
                    state <= FETCH_INSTR;
                end
            endcase
        end
    end

    assign mem_addr = PC;
    assign mem_rstrb = (state == FETCH_INSTR);

    // Debug info
    `ifdef BENCH
        always @(posedge clk) begin
            if (state == FETCH_REGS) begin
                $display("PC=%0d, INSTR=%b_%b_%b", PC, instr[31:12], instr[11:7], instr[6:0]);
                case (1'b1)
                    isALUreg:
                        $display("ALUreg r%0d <- r%0d(%0d) r%0d(%0d) funct3=%b", rdId, rs1Id, rs1, rs2Id, rs2, funct3);
                    isALUimm:
                        $display("ALUimm r%0d <- r%0d(%0d) imm=%0d funct3=%b", rdId, rs1Id, rs1, Iimm, funct3);
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
                $display("aluOut=%0d", aluOut);
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
    Clockworks #(
        .SLOW(21) // Divide clock frequency by 2^21
    )CW(
        .CLK(CLK),
        .RESET(RESET),
        .clk(clk),
        .resetn(resetn)
    );
    wire clk;    // internal clock
    wire resetn; // internal reset signal, goes low on reset

   Memory RAM(
        .clk(clk),
        .mem_addr(mem_addr),
        .mem_rdata(mem_rdata),
        .mem_rstrb(mem_rstrb)
   );
    wire [31:0] mem_addr;
    wire [31:0] mem_rdata;
    wire mem_rstrb;

   Processor CPU(
      .clk(clk),
      .resetn(resetn),		 
      .mem_addr(mem_addr),
      .mem_rdata(mem_rdata),
      .mem_rstrb(mem_rstrb),
      .x1(x1)		 
   );
    wire [31:0] x1;

    // Plug the leds on register 1 to see its contents
    assign LEDS = x1[4:0];
    assign TXD  = 1'b0; // not used for now
endmodule
