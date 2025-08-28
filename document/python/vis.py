import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import pandas as pd
import threading
import time

# Konfiguration
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
MAX_DATA_POINTS = 2400

# Namen für die vier Werte
VALUE_NAMES = ['AVDD', 'DVDD', 'Relation', 'Resistance']

# Globale Variablen für Daten, Serial-Port und Referenzwerte
data = {name: {'x_time': [], 'y_value': []} for name in VALUE_NAMES}
data_buffer = []
ser = None
is_running = True

# Speichert die ersten empfangenen Werte als Referenz
reference_values = {'Relation': None, 'Resistance': None}
reference_lines = {'Relation': None, 'Resistance': None}

# NEU: Puffer für die gesamte Sitzungsdatenspeicherung
all_session_data = {name: {'x_time': [], 'y_value': []} for name in VALUE_NAMES}

# --- Serial-Lesefunktion ---
def read_serial_data():
    global ser, is_running
    print("Starte Serial-Lesethread...")
    
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Erfolgreich verbunden mit {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"Fehler beim Verbinden mit {SERIAL_PORT}: {e}")
        is_running = False
        return

    while is_running:
        if ser and ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    parts = line.split(',')
                    if len(parts) == len(VALUE_NAMES):
                        values = []
                        valid_data = True
                        for i in range(len(parts)):
                            part = parts[i]
                            try:
                                if i == 2:
                                    values.append(float(part.strip()) / values[1])
                                else:
                                    values.append(float(part.strip()))
                            except (ValueError, IndexError) as e:
                                print(f"Ungültiger Wert: '{part}' ist kein gültiger Float. Fehler: {e}")
                                valid_data = False
                                break
                        
                        if valid_data:
                            # Speichere die Werte im Echtzeit-Puffer
                            data_buffer.append({'timestamp': time.time(), 'values': values})
            except Exception as e:
                print(f"Fehler beim Lesen: {e}")
        time.sleep(0.01)

    if ser:
        ser.close()
        print(f"Verbindung zu {SERIAL_PORT} geschlossen.")

# --- Plot-Aktualisierungsfunktion ---
def update_plot(frame, lines, axes):
    if data_buffer:
        new_data_points = data_buffer.copy()
        data_buffer.clear()
        
        for d_point in new_data_points:
            timestamp = d_point['timestamp']
            values = d_point['values']
            
            for i, name in enumerate(VALUE_NAMES):
                # Füge Daten für den Plot hinzu (begrenzt)
                data[name]['x_time'].append(timestamp)
                data[name]['y_value'].append(values[i])
                
                # Füge Daten für die gesamte Sitzung hinzu (unbegrenzt)
                all_session_data[name]['x_time'].append(timestamp)
                all_session_data[name]['y_value'].append(values[i])
                
                # Prüfe, ob der Name im reference_values Dictionary existiert
                if name in reference_values and reference_values[name] is None:
                    reference_values[name] = values[i]
                    line_index = VALUE_NAMES.index(name)
                    reference_lines[name] = axes[line_index].axhline(y=reference_values[name], color='r', linestyle='--', label='Referenzwert')
                    axes[line_index].legend()

        for name in VALUE_NAMES:
            if len(data[name]['x_time']) > MAX_DATA_POINTS:
                data[name]['x_time'] = data[name]['x_time'][-MAX_DATA_POINTS:]
                data[name]['y_value'] = data[name]['y_value'][-MAX_DATA_POINTS:]

        for i, name in enumerate(VALUE_NAMES):
            lines[i].set_data(data[name]['x_time'], data[name]['y_value'])
            ax = lines[i].axes
            ax.relim()
            ax.autoscale_view()
    
    all_artists = lines + [line for line in reference_lines.values() if line is not None]
    return all_artists

# --- Tastendruck-Funktion zum Zurücksetzen der Referenzwerte ---
def on_press(event):
    if event.key == 'r':
        print("Tastendruck 'r' erkannt. Setze Referenzwerte zurück...")
        
        for name, line in reference_lines.items():
            if line is not None:
                line.remove()
                reference_lines[name] = None
        
        for name in reference_values:
            reference_values[name] = None
        
        plt.draw()

# --- Hauptfunktion ---
def main():
    global is_running
    
    serial_thread = threading.Thread(target=read_serial_data, daemon=True)
    serial_thread.start()

    fig, axes = plt.subplots(len(VALUE_NAMES), 1, figsize=(10, 12))
    lines = []
    
    for i, name in enumerate(VALUE_NAMES):
        ax = axes[i]
        line, = ax.plot([], [], label=name)
        lines.append(line)
        ax.set_title(f'Echtzeit-Daten: {name}')
        ax.set_xlabel('Zeit (s)')
        ax.set_ylabel('Wert')
        ax.grid(True)
        ax.legend()
    
    plt.tight_layout()

    fig.canvas.mpl_connect('key_press_event', on_press)

    ani = FuncAnimation(fig, update_plot, fargs=(lines, axes), interval=50, blit=False, cache_frame_data=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("Programm wird beendet...")
    finally:
        is_running = False
        serial_thread.join()
        
        # NEU: Speichere die gesamten Sitzungsdaten
        print("Sammle alle Sitzungsdaten zum Speichern...")
        df_data = {'timestamp': []}
        for name in VALUE_NAMES:
            df_data[f'{name}_Wert'] = all_session_data[name]['y_value']
            # Wir nehmen an, dass die Zeitstempel für alle Werte gleich sind,
            # was bei kommagetrennten Werten pro Zeile der Fall ist.
            if not df_data['timestamp']:
                df_data['timestamp'] = all_session_data[name]['x_time']

        all_data_df = pd.DataFrame(df_data)
        
        file_name = 'serielle_daten_4_werte.csv'
        all_data_df.to_csv(file_name, index=False)
        print(f"Daten erfolgreich in '{file_name}' gespeichert.")

if __name__ == '__main__':
    main()