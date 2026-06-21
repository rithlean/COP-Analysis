import COP

ports_in, ports_out, instances, assigns, pin_to_net = COP.parse_verilog("netlist/ISCAS89/s38417.v")
cc1_vals, co_vals = COP.run_cop(ports_in, ports_out, instances, assigns)

targets = ["n3257", "n3269", "n3910", "n3261"]
for net in targets:
    print(net, "CC1=%.4f" % cc1_vals.get(net, -1), "CO=%.6f" % co_vals.get(net, -1))

def find_driver_and_sinks(net, instances):
    driver = None
    sinks = []
    for inst in instances:
        for port, n in inst["conn"].items():
            if n == net:
                if port in ("Y","Q","QN","S","CO","SO"):
                    driver = (inst["inst"], inst["cell"], port)
                else:
                    sinks.append((inst["inst"], inst["cell"], port))
    return driver, sinks

for net in targets:
    drv, sinks = find_driver_and_sinks(net, instances)
    print("\n", net, "driver=", drv)
    print("   sinks=", sinks)
