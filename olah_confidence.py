import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
df = pd.read_csv('/home/pambudi/Yolov8/Data/General/data_10:07_05Aug.csv')

# Filter the rows where the class is 'end-track'
track_df = df[df['Class'] == 'track']
end_track_df = df[df['Class'] == 'end-track']

# Extract the confidence scores and calculate the average
confidence_scores1 = end_track_df['Confidence Score'].apply(lambda x: float(x.strip('tensor([').strip('])')))
average_confidence_score1 = confidence_scores1.mean()

confidence_scores2 = end_track_df['Confidence Score'].apply(lambda x: float(x.strip('tensor([').strip('])')))
average_confidence_score2 = confidence_scores2.mean()

print(f'Average Confidence Score for track: {average_confidence_score1:.2f}')
print(f'Average Confidence Score for end-track: {average_confidence_score2:.2f}')


