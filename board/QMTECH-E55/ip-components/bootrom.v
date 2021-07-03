// megafunction wizard: %ROM: 1-PORT%
// GENERATION: STANDARD
// VERSION: WM1.0
// MODULE: altsyncram 

// synopsys translate_off
`timescale 1 ps / 1 ps
// synopsys translate_on
module boot_rom (
   address,
   clock,
   q);

   input   [8:0]  address;
   input     clock;
   output   [15:0]  q;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_off
`endif
   tri1     clock;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_on
`endif

   wire [15:0] sub_wire0;
   wire [15:0] q = sub_wire0[15:0];

   altsyncram   altsyncram_component (
            .address_a (address),
            .clock0 (clock),
            .q_a (sub_wire0),
            .aclr0 (1'b0),
            .aclr1 (1'b0),
            .address_b (1'b1),
            .addressstall_a (1'b0),
            .addressstall_b (1'b0),
            .byteena_a (1'b1),
            .byteena_b (1'b1),
            .clock1 (1'b1),
            .clocken0 (1'b1),
            .clocken1 (1'b1),
            .clocken2 (1'b1),
            .clocken3 (1'b1),
            .data_a ({16{1'b1}}),
            .data_b (1'b1),
            .eccstatus (),
            .q_b (),
            .rden_a (1'b1),
            .rden_b (1'b1),
            .wren_a (1'b0),
            .wren_b (1'b0));
   defparam
      altsyncram_component.address_aclr_a = "NONE",
      altsyncram_component.clock_enable_input_a = "BYPASS",
      altsyncram_component.clock_enable_output_a = "BYPASS",
      altsyncram_component.init_file = "../../rom/m9312/bootrom.mif",
      altsyncram_component.intended_device_family = "Cyclone IV E",
      altsyncram_component.lpm_hint = "ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=boot_rom",
      altsyncram_component.lpm_type = "altsyncram",
      altsyncram_component.numwords_a = 512,
      altsyncram_component.operation_mode = "ROM",
      altsyncram_component.outdata_aclr_a = "NONE",
      altsyncram_component.outdata_reg_a = "UNREGISTERED",
      altsyncram_component.widthad_a = 9,
      altsyncram_component.width_a = 16,
      altsyncram_component.width_byteena_a = 1;


endmodule
