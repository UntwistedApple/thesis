import pandas as pd
import matplotlib.pyplot as plt

def visualize_csv_data(file_name='filtered.csv'):
    """
    Visualisiert die in einer CSV-Datei gespeicherten seriellen Daten.

    Args:
        file_name (str): Der Name der zu visualisierenden CSV-Datei.
    """
    try:
        # Lese die CSV-Datei in einen Pandas DataFrame ein
        df = pd.read_csv(file_name)
        print(f"Daten aus '{file_name}' erfolgreich eingelesen.")
        print(df.head()) # Zeige die ersten Zeilen zur Überprüfung
        
    except FileNotFoundError:
        print(f"Fehler: Datei '{file_name}' nicht gefunden.")
        return
    except Exception as e:
        print(f"Ein Fehler ist beim Einlesen der Datei aufgetreten: {e}")
        return

    # Extrahiere die Spaltennamen für die Werte
    value_cols = [col for col in df.columns if col.endswith('_Wert')]
    
    # Sicherstellen, dass die Spalten in der richtigen Reihenfolge sind
    plot_titles = [col.replace('_Wert', '') for col in value_cols]

    # Erstelle die Plots
    fig, axes = plt.subplots(len(value_cols), 1, figsize=(10, 12))
    
    # Wenn es nur einen Subplot gibt, sorge dafür, dass axes eine Liste ist
    if len(value_cols) == 1:
        axes = [axes]

    # Nutze die 'timestamp'-Spalte, die in der CSV-Datei existiert
    time_col_name = 'timestamp'
    
    for i, col_name in enumerate(value_cols):
        ax = axes[i]
        
        # Datenbereinigung ist hier nicht mehr nötig, da die 'timestamp'-Spalte immer vorhanden ist
        # und die Daten bereits korrekt ausgerichtet sind.
        
        # Plotten der Daten
        ax.plot(df[time_col_name], df[col_name], label=col_name.replace('_Wert', ''))
        
        ax.set_title(f'Daten: {plot_titles[i]}')
        ax.set_xlabel('Zeit (s)')
        ax.set_ylabel('Wert')
        ax.grid(True)
        ax.legend()
    
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    visualize_csv_data()