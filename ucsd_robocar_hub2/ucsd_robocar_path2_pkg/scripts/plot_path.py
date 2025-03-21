import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("../paths/path_from_poses.csv")
df = df.dropna(how='any')

plt.plot(df['px'].values, df['py'].values)
plt.title('Trajectory Recorded')
plt.xlabel('x')
plt.ylabel('y')
plt.show()
