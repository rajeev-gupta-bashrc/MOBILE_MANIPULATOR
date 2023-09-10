input_file = '/home/rajeev/Untitled_1.xlsx'
import pandas as pd

# Replace 'input_file.xlsx' with the path to your input Excel file

# Load the Excel file into a Pandas DataFrame
df = pd.read_excel(input_file)

# Replace empty cells with an empty string
df = df.fillna('')

# Define a function to remove trailing single quotation marks
def remove_trailing_quote(cell_value):
    if isinstance(cell_value, str) and cell_value.endswith("'"):
        return float(cell_value[:-1])  # Remove the trailing single quotation mark
    return float(cell_value)

# Apply the remove_trailing_quote function to each cell in the DataFrame

# Create a new DataFrame to store the split data
new_df = pd.DataFrame()

# Iterate through each column in the original DataFrame
for column in df.columns:
    # Split the data in each column on space and expand it into separate columns
    split_data = df[column].str.split(expand=True)
    
    # Add the split data to the new DataFrame
    new_df = pd.concat([new_df, split_data], axis=1)

# Replace NaN values with an empty string if needed
new_df = new_df.fillna('')

# Replace 'output_file.xlsx' with the desired path and file name for the new Excel file

new_df = new_df.applymap(remove_trailing_quote)
output_file = '/home/rajeev/output_file.xlsx'
# Save the new DataFrame to a new Excel file without the index
new_df.to_excel(output_file, index=False, header=False)

print("New Excel file saved successfully.")

