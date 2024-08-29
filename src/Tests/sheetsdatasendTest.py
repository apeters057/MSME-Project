import gspread
from oauth2client.service_account import ServiceAccountCredentials

# Set the scope and credentials
scope = ['https://spreadsheets.google.com/feeds', 'https://www.googleapis.com/auth/drive']
credentials = ServiceAccountCredentials.from_json_keyfile_name('/home/apeters/project/env/src/Tests/credentials.json', scope)
gc = gspread.authorize(credentials)

# Open the Google Spreadsheet using its title
spreadsheet = gc.open('Voltage_SpeedData')

# Select the worksheet
worksheet = spreadsheet.sheet1 

# Update cell values (Row , Column, Value)
# Title Cells
worksheet.update_cell(1, 1, 'Time')
worksheet.update_cell(1, 2, 'Input - Voltage')
worksheet.update_cell(1, 3, 'Output - Speed')

print('Data sent to Google Sheets!')
