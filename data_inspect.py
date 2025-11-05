#Purpose of this file is to inspect the content of uncleaned csv file

import pandas as pd

df = pd.read_csv('data.csv')
print(df.head())
print(df.info())
print(df.columns)