import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, AutoMinorLocator
from scipy.io import wavfile
from scipy.signal import correlate
from scipy import signal
from scipy.signal import resample, find_peaks, medfilt
from scipy.ndimage import gaussian_filter1d


TimeConv = 1


class BrownianNoiseGenerator:
    def __init__(self, seed=0x3701):
        self.lfsr = seed if seed != 0 else 1  # Avoid zero state
        self.brownian_state = 2048  # Start in the middle of the 12-bit range

    def white_noise(self):
        """Generate a 12-bit pseudo-random number using LFSR."""
        self.lfsr ^= (self.lfsr >> 7) & 0xFFFF
        self.lfsr ^= (self.lfsr << 9) & 0xFFFF
        self.lfsr ^= (self.lfsr >> 13) & 0xFFFF
        return self.lfsr & 0x0FFF  # Return 12-bit value (0-4095)

    def generate_new_seed(self):
        """Generate a new LFSR seed."""
        self.lfsr ^= 0xB400
        if self.lfsr == 0:
            self.lfsr = 1

    def generate_sequence(self):
        """Generate a sequence of 12-bit Brownian noise."""
        seq_array = []
        
        for _ in range(4096):
            white_noise = self.white_noise()
            step = (white_noise & 0x7F) - 64  # Small step (-64 to +63)
            
            if white_noise & 1:
                step += 1  # Ensure symmetry
            
            correction = (2048 - self.brownian_state) // 256  # Small force toward center
            step += correction
            
            self.brownian_state += step  # Integrate step
            
            self.brownian_state = max(0, min(4095, self.brownian_state))
            seq_array.append(self.brownian_state)
        
        self.brownian_state = 2048
        self.generate_new_seed()  # Create new seed for the next generation
        
        return np.array(seq_array, dtype=np.float64)

    def set_seed(self, seed):
        """Set a new LFSR seed (avoiding zero)."""
        self.lfsr = seed if seed != 0 else 1
    
    def upsample(self, noise_sequence, input_rate=10000, output_rate=50000):
        """Upsample the noise sequence using linear interpolation."""
        factor = output_rate / input_rate
        upsampled_length = int(len(noise_sequence) * factor)
        return resample(noise_sequence, upsampled_length)



def detect_high_peaks(signal, height_offset=0.9):

    
    signal = abs(signal)
    signal = medfilt(signal, 27)
    signal = gaussian_filter1d(signal, 13)
    
    peaks, properties = find_peaks(signal, distance=2000)
    
    if len(peaks) == 0:
        return [], {}
    
    peak_heights = properties['peak_heights'] if 'peak_heights' in properties else signal[peaks]
    mean_height = np.mean(signal)
    std_height = np.std(signal)
    max_height = max(peak_heights)
    threshold = 0.9*max_height
    

    
    high_peaks = peaks[peak_heights > threshold]
    if(len(high_peaks) > 1): #If there were more peaks found then it must be noisy and we need to discard
        return signal, [], properties
    return signal, high_peaks, properties

def PulseDetect(FileName, NoisePulse):
    global TimeConv
    
    # Read audio file
    Fs, AudioSig = wavfile.read(FileName)
    AudioSig = AudioSig.astype(np.float64)
    AudioSig = AudioSig
    
    
    NoisePulse = generator.upsample(NoisePulse, input_rate=10000, output_rate=Fs) #Upsample to frequency of the measured signal
    NoisePulse = (NoisePulse/4096)*3.3 # Normalize
    
    TimeConv = (1/Fs)*1000
    
    NoisePulse = NoisePulse - np.mean(NoisePulse) #remove DC Bias
    AudioSig = AudioSig - np.mean(AudioSig) #remove DC Bias
    
    # Perform cross-correlation
    CorrSig = correlate(AudioSig, NoisePulse, mode='full', method='direct')
    
    CorrSig, high_peaks, _ = detect_high_peaks(CorrSig, 0.9)
    
    return CorrSig, high_peaks, AudioSig
    

def ShiftSignals(signal, shift_val):
    shifted_signal = np.roll(signal, shift_val)
    
    if shift_val > 0: #Signal shifted to the right
        shifted_signal[:shift_val] = 0
    elif shift_val < 0: #signal shifted to the left
        shifted_signal[len(shifted_signal)-shift_val:] = 0
    else:
        pass
    
    return shifted_signal

def AllignSignals(sig1, sig2, peaks1, peaks2): #TODO make it so that you can allign multiple signals

    shift = peaks1[-1] - peaks2[-1]  # Compute shift based on the last detected peak
    
    sig2 = ShiftSignals(sig2, shift)
    
    return sig1, sig2


def sweep_delay(sig1, sig2, fs, refine=True):


    # remove any DC offset
    x1 = sig1 - np.mean(sig1)
    x2 = sig2 - np.mean(sig2)

    # full cross-correlation
    corr = correlate(x2, x1, mode='full')

    # lag axis: from -(N-1) to +(N-1)
    n = len(x1)
    lags = np.arange(-n+1, n)

    # index of max correlation
    i_peak = np.argmax(corr)
    lag = lags[i_peak]

    delay_s = lag / fs
    return delay_s, lag

# Example Usage
if __name__ == "__main__":
    # Instantiate the generator with a specific seed
    generator = BrownianNoiseGenerator(seed=0x3701)  # Set the same seed as in C

    
    MainSig = []
    AllignSig = []
    CorrSig1 = []
    CorrSig2 = []
    
    
    
    for i in range(0,20): # go over 10 possible noises sequences, if it exists align and quit
        NoisePulse = generator.generate_sequence()# Normalize
        
        
        #CorrSig1, high_peaks1, AudioSig1 = PulseDetect("meas_sweep_v1_left.wav", NoisePulse)
        #CorrSig2, high_peaks2, AudioSig2 = PulseDetect("meas_sweep_v1_right.wav", NoisePulse)
        
        #CorrSig1, high_peaks1, AudioSig1 = PulseDetect("meas_sweep_v3_Phase_Left.wav", NoisePulse)
        #CorrSig2, high_peaks2, AudioSig2 = PulseDetect("meas_sweep_v3_Phase_Right.wav", NoisePulse)
        
        CorrSig1, high_peaks1, AudioSig1 = PulseDetect("./RecorderRecordings/Left/record_left07.WAV", NoisePulse)
        CorrSig2, high_peaks2, AudioSig2 = PulseDetect("./RecorderRecordings/Right/record_right07.WAV", NoisePulse)
        
        print("#############################################")
        print("Trying to find the signal N-try:" + str(i))
        print("Location of peaks from signal 1: " + str(high_peaks1))
        print("Location of peaks from signal 2: " + str(high_peaks2))
        try:
            if len(high_peaks1) != 0 and len(high_peaks2) != 0:
                if len(high_peaks1) < len(high_peaks2):
                    high_peaks1 = np.pad(high_peaks1, (len(high_peaks2) - len(high_peaks1), 0))
                elif len(high_peaks2) < len(high_peaks1):
                    high_peaks2 = np.pad(high_peaks2, (len(high_peaks1) - len(high_peaks2), 0))

        except:
            print("Delta of the signals: " + "No deltas found")
            
        
        if len(high_peaks1) != 0 and len(high_peaks2) != 0:
            MainSig, AllignSig = AllignSignals(AudioSig1, AudioSig2, high_peaks1, high_peaks2)
            CorrSig1, CorrSig2 = AllignSignals(CorrSig1, CorrSig2, high_peaks1, high_peaks2)
            
            delay_s, lag = sweep_delay(MainSig, AllignSig, 50000, refine=True)
            print("Delay(ms): " + str(delay_s*1000) + " Lag(samples): " + str(lag))
            
            print("Found the allignment in N-tries: " + str(i))
            
            break
            pass
        else:
            MainSig = []
            AllignSig = []
            CorrSig1 = []
            CorrSig2 = []
        pass
    
    # Plotting
    
    t_main = np.arange(len(MainSig)) * TimeConv
    t_alling = np.arange(len(AllignSig)) * TimeConv
    t_corr1 = np.arange(len(CorrSig1)) * TimeConv
    t_corr2 = np.arange(len(CorrSig2)) * TimeConv
    
    fig, axs = plt.subplots(4, 1, figsize=(10, 8))
    
    major_spacing = 1000  # major ticks every 100 ms
    for ax in axs.flatten():
        ax.grid(which='major', linestyle='-', linewidth=0.5, color='gray')
        
    
    CorrSig1DB = CorrSig1 / np.max(CorrSig1)
    CorrSig1DB   = 20 * np.log10(CorrSig1DB + 1e-12)
    
    CorrSig2DB = CorrSig2 / np.max(CorrSig2)
    CorrSig2DB   = 20 * np.log10(CorrSig2DB + 1e-12)
    
    axs[0].plot((NoisePulse/4098) * 3.3)
    axs[0].set_title("NoisePulse")
    axs[0].set_xlabel("samples (N)")
    axs[0].set_ylabel("Voltage (V)")
    
    axs[1].plot(t_main, MainSig)
    axs[1].plot(t_alling, AllignSig, alpha=0.7)
    axs[1].set_title("AudioSig")
    axs[1].set_xlabel("time (ms)")
    axs[1].set_ylabel("Voltage (V)")
    
    axs[2].plot(t_corr1, CorrSig1)
    axs[2].plot(t_corr2, CorrSig2, alpha=0.7)
    axs[2].set_title("CorrSig")
    axs[2].set_xlabel("time (ms)")
    axs[2].set_ylabel("AU")
    
    axs[3].plot(t_corr1, CorrSig1DB)
    axs[3].plot(t_corr2, CorrSig2DB, alpha=0.7)
    axs[3].set_title("CorrSig")
    axs[3].set_xlabel("time (ms)")
    axs[3].set_ylabel("dB")
    
    
    plt.tight_layout()
    plt.show()
    
    
# =============================================================================
#     # Generate the spectrogram
#     frequencies, times, spectrogram = signal.spectrogram(AudioSig[0:300000], Fs)
#     # Plot the spectrogram
#     plt.figure(figsize=(10, 4))
#     plt.pcolormesh(times, frequencies, 10 * np.log10(spectrogram), shading='gouraud')
#     plt.ylabel('Frequency [Hz]')
#     plt.xlabel('Time [sec]')
#     plt.title('Spectrogram')
#     plt.colorbar(label='Intensity [dB]')
#     plt.show()
# =============================================================================
    
    # Playing the audio (requires simpleaudio or sounddevice)

    # import sounddevice as sd
    # sd.play(AudioSig / np.max(np.abs(AudioSig)), samplerate=Fs)

