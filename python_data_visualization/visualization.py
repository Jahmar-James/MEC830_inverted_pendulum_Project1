from SerialPort import SerialPort, DataCollector
from plotter import Plotter, PIDAnalysisPlotter

def main():
    serial_port = SerialPort("COM5", 9600)
    serial_port.connect()

    data_collector = DataCollector(serial_port)
    Fig_1_plotter =  Plotter()
    # Fig_2_plotter = PIDAnalysisPlotter()

    try:
        while True:
            angles,distances, set_points, pid_outputs, errors, cumulative_errors, derivatives = data_collector.collect()
            Fig_1_plotter.update(distances, set_points, errors)
            # Fig_2_plotter.update(pid_outputs, angles)
    
    except KeyboardInterrupt:
        serial_port.data.close()
        Fig_1_plotter.show()
        # Fig_2_plotter.show()

if __name__ == "__main__":
    main()