module test_patch (TPEnable, n330, n277);
input TPEnable, n330, n277;
wire _sp0_ctrl;
wire _sp0_out;

and _sp0_en (_sp0_ctrl, n277, TPEnable);
or _sp0_gate (_sp0_out, n330, _sp0_ctrl);

endmodule