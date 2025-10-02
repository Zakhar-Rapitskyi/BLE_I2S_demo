import wave
import numpy as np
import os
import glob

def list_audio_files(directory="."):
    """List all WAV files in current directory"""
    wav_files = glob.glob(os.path.join(directory, "*.wav"))
    return wav_files

def get_user_choice(files):
    """Get user choice from file list"""
    print("\nAvailable audio files:")
    for i, file in enumerate(files, 1):
        file_size = os.path.getsize(file) // 1024  # Size in KB
        print(f"  {i}. {os.path.basename(file)} ({file_size} KB)")
    
    while True:
        try:
            choice = int(input(f"\nSelect file (1-{len(files)}): "))
            if 1 <= choice <= len(files):
                return files[choice - 1]
            else:
                print(f"Please enter a number between 1 and {len(files)}")
        except ValueError:
            print("Please enter a valid number")

def get_array_name(default_name):
    """Get array name from user"""
    name = input(f"Enter array name [{default_name}]: ").strip()
    return name if name else default_name

def get_output_filename(default_name):
    """Get output filename from user"""
    name = input(f"Enter output header filename [{default_name}]: ").strip()
    return name if name else default_name

def convert_wav_to_c(input_wav, output_h, array_name):
    """Convert WAV file to C header file"""
    try:
        with wave.open(input_wav, "rb") as wav:
            channels = wav.getnchannels()
            sampwidth = wav.getsampwidth()
            framerate = wav.getframerate()
            n_frames = wav.getnframes()
            
            print(f"\nFile: {input_wav}")
            print(f"Channels: {channels}, Sample width: {sampwidth} bytes")
            print(f"Sample rate: {framerate} Hz, Frames: {n_frames}")
            print(f"Duration: {n_frames / framerate:.2f} seconds")
            
            frames = wav.readframes(n_frames)
            
            if sampwidth == 1:  # 8-bit unsigned (0-255)
                samples = np.frombuffer(frames, dtype=np.uint8)
                samples = ((samples.astype(np.int32) - 128) * 256).astype(np.int16)
                print("Converted 8-bit unsigned to 16-bit signed")
                
            elif sampwidth == 2:  # 16-bit signed
                samples = np.frombuffer(frames, dtype=np.int16)
                
            elif sampwidth == 3:  # 24-bit
                # Convert 24-bit to 16-bit
                samples_24bit = np.frombuffer(frames, dtype=np.uint8).reshape(-1, 3)
                samples = np.zeros(len(samples_24bit), dtype=np.int16)
                for i, sample in enumerate(samples_24bit):
                    # Combine 3 bytes into 32-bit integer, then shift to 16-bit
                    sample_32bit = (sample[0] << 8) | (sample[1] << 16) | (sample[2] << 24)
                    samples[i] = (sample_32bit >> 16).astype(np.int16)
                print("Converted 24-bit to 16-bit signed")
                
            else:
                raise ValueError(f"Unsupported sample width: {sampwidth} bytes")
            
            # Handle multi-channel audio
            if channels == 2:
                samples = samples[::2]  # Take left channel
                print("Extracted left channel from stereo")
            elif channels > 2:
                samples = samples[::channels]  # Take first channel
                print(f"Extracted first channel from {channels}-channel audio")
            
            # Ensure output directory exists
            os.makedirs(os.path.dirname(output_h) if os.path.dirname(output_h) else ".", exist_ok=True)
            
            with open(output_h, "w") as f:
                f.write(f"#ifndef {array_name.upper()}_H\n")
                f.write(f"#define {array_name.upper()}_H\n\n")
                f.write(f"#define {array_name.upper()}_SIZE {len(samples)}\n\n")
                f.write(f"static const int16_t {array_name}[{len(samples)}] __attribute__((aligned(4))) = {{\n    ")
                
                for i, s in enumerate(samples):
                    f.write(f"{s}")
                    if i < len(samples) - 1:
                        f.write(", ")
                    if (i + 1) % 10 == 0:  # 10 samples per line
                        f.write("\n    ")
                
                f.write("\n};\n\n")
                f.write(f"#endif // {array_name.upper()}_H\n")
            
            print(f"\n✅ Generated {output_h}")
            print(f"📊 Array size: {len(samples)} samples = {len(samples) * 2} bytes")
            print(f"⏱️  Duration at 16kHz: {len(samples) / 16000:.2f} seconds")
            
    except Exception as e:
        print(f"❌ Error processing file: {e}")

def main():
    """Main interactive function"""
    print("🎵 WAV to C Header Converter")
    print("=" * 40)
    
    # List available files
    files = list_audio_files()
    
    if not files:
        print("No WAV files found in current directory.")
        manual_input = input("Enter path to WAV file (or press Enter to exit): ").strip()
        if manual_input and os.path.exists(manual_input):
            files = [manual_input]
        else:
            print("No files to process. Exiting.")
            return
    
    # Let user choose file
    selected_file = get_user_choice(files)
    
    # Generate default names based on filename
    base_name = os.path.splitext(os.path.basename(selected_file))[0]
    default_array_name = f"{base_name}_data".replace("-", "_").replace(" ", "_")
    default_output_file = f"{base_name}.h"
    
    # Get user preferences
    array_name = get_array_name(default_array_name)
    output_file = get_output_filename(default_output_file)
    
    # Ensure .h extension
    if not output_file.lower().endswith('.h'):
        output_file += '.h'
    
    # Confirm conversion
    print(f"\nConversion settings:")
    print(f"  Input:  {selected_file}")
    print(f"  Output: {output_file}")
    print(f"  Array:  {array_name}")
    
    confirm = input("\nProceed with conversion? (y/n): ").strip().lower()
    if confirm in ['y', 'yes']:
        convert_wav_to_c(selected_file, output_file, array_name)
    else:
        print("Conversion cancelled.")

def batch_convert():
    """Batch convert all WAV files in directory"""
    files = list_audio_files()
    
    if not files:
        print("No WAV files found in current directory.")
        return
    
    print(f"Found {len(files)} WAV files:")
    for file in files:
        print(f"  - {os.path.basename(file)}")
    
    confirm = input("\nConvert all files? (y/n): ").strip().lower()
    if confirm not in ['y', 'yes']:
        return
    
    for file in files:
        base_name = os.path.splitext(os.path.basename(file))[0]
        array_name = f"{base_name}_data".replace("-", "_").replace(" ", "_")
        output_file = f"{base_name}.h"
        
        print(f"\nConverting {file}...")
        convert_wav_to_c(file, output_file, array_name)

if __name__ == "__main__":
    print("WAV to C Header Converter")
    print("1. Interactive conversion (choose file)")
    print("2. Batch convert all WAV files")
    
    choice = input("Select mode (1/2): ").strip()
    
    if choice == "2":
        batch_convert()
    else:
        main()
    
    input("\nPress Enter to exit...")