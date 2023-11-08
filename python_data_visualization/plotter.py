from matplotlib import pyplot as plt
import seaborn as sns
import numpy as np

class Plotter:
    def __init__(self):
        sns.set_style("whitegrid")
        self.fig, self.axs = plt.subplots(2, 1, figsize=(8, 8))
        self.ax1, self.ax2 = self.axs
        self._setup_main_plot()
        self._setup_histogram_plot()
        plt.tight_layout()
        plt.ion()

    def _setup_main_plot(self):
        self.line1, = self.ax1.plot([], [], 'b-', label="Distance")
        self.line2, = self.ax1.plot([], [], 'r-', label="Set Point")
        self.line3, = self.ax1.plot([], [], 'g--', label="Absolute Error")
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Distance (cm)')
        self.ax1.set_title('Distance vs. Set Point Over Time')
        self.ax1.legend(loc="upper left")
        self.ax1.set_ylim(0, 20)

    def _setup_histogram_plot(self):
        self.ax2.set_title('Histogram of Errors')
        self.ax2.set_xlabel('Error')
        self.ax2.set_ylabel('Frequency')

    def update(self, distances, set_points, errors):
        # Data for plotting
        display_limit = 100
        plot_distances = distances[-display_limit:]
        plot_setpoints = set_points[-display_limit:]
        plot_errors = errors[-display_limit:]
        
        # Update Main Plot
        self.line1.set_ydata(plot_distances)
        self.line1.set_xdata(list(range(len(plot_distances))))
        self.line2.set_ydata(plot_setpoints)
        self.line2.set_xdata(list(range(len(plot_setpoints))))
        self.line3.set_ydata(np.abs(plot_errors))
        self.line3.set_xdata(list(range(len(plot_errors))))
        
        # Update Histogram
        self.ax2.clear()
        sns.histplot(errors, ax=self.ax2, bins=30, kde=True)
        self.ax2.set_title('Histogram of Errors')
        self.ax2.set_xlabel('Error')
        self.ax2.set_ylabel('Frequency')
        
        # Adjust x-axis limits
        self.ax1.set_xlim(0, max(display_limit, len(plot_distances)))
        
        plt.pause(0.1)


    def show(self):
        plt.show()

class PIDAnalysisPlotter:
    def __init__(self):
        sns.set_style("whitegrid")
        self.fig, self.axs = plt.subplots(3, 1, figsize=(8, 10))
        self.ax1, self.ax2, self.ax3 = self.axs
        self._setup_pid_plot()
        self._setup_angle_plot()
        self._setup_fft_plot()
        plt.tight_layout()
        plt.ion()

    def _setup_pid_plot(self):
        self.line1, = self.ax1.plot([], [], 'm-', label="PID Output")
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('PID Output')
        self.ax1.set_title('PID Output Over Time')
        self.ax1.legend(loc="upper left")

    def _setup_angle_plot(self):
        self.line2, = self.ax2.plot([], [], 'c-', label="Servo Angle")
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('Angle (degrees)')
        self.ax2.set_title('Servo Angle Over Time')
        self.ax2.legend(loc="upper left")

    def _setup_fft_plot(self):
        self.line3, = self.ax3.plot([], [], 'b-', label="FFT Magnitude")
        self.ax3.set_xlabel('Frequency (Hz)')
        self.ax3.set_ylabel('Magnitude')
        self.ax3.set_title('Frequency Analysis of PID Output')
        self.ax3.legend(loc="upper left")

    def update(self, pid_outputs, angles):
        # Update PID Plot
        self.line1.set_ydata(pid_outputs)
        self.line1.set_xdata(list(range(len(pid_outputs))))
        
        # Update Angle Plot
        self.line2.set_ydata(angles)
        self.line2.set_xdata(list(range(len(angles))))
        
        # Only compute FFT if pid_outputs is not empty
        if pid_outputs:
            #Check PID Output is long enough to compute FFT

            fft_output = np.abs(np.fft.fft(pid_outputs))[:len(pid_outputs)//2]
            freqs = np.fft.fftfreq(len(pid_outputs), 0.1)[:len(pid_outputs)//2]
            self.line3.set_ydata(fft_output)
            self.line3.set_xdata(freqs)
        
        plt.pause(0.1)

    def show(self):
        plt.show()
