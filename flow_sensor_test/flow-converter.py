import pandas as pd
import matplotlib.pyplot as plt

# load data
df = pd.read_csv('flow-sensor.csv')

print(list(df.columns))

df['Time_s'] = df['Time'] / 1000.0 # convert milliseconds to seconds

# conversion function, approximate linear fits for the two regions of
def convert_to_slpm(voltage):
    if voltage > 120.2:
        return (voltage - 129.2) / -9
    else:
        return (voltage - 129.2) / -3.5

df['SLPM'] = df['FlowSensor'].apply(convert_to_slpm)

plt.plot(df['Time_s'], df['SLPM'])
plt.title('Flow Rate Over Time')
plt.xlabel('Time (s)')
plt.ylabel('Flow Rate (in SLPM)')
plt.show()