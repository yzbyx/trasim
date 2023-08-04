import matplotlib.pyplot as plt
import pandas as pd

from python.data_plot import Plot

df = pd.read_csv("D:\\test.txt", sep="\t")
Plot.basic_plot(lane_id=0, data_df=df)
Plot.spatial_time_plot(car_id=-1, lane_add_num=0, data_df=df)
Plot.show()
