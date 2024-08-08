import csv

def filter_csv(input_file, output_file):
    with open(input_file, mode='r', newline='') as infile, \
         open(output_file, mode='w', newline='') as outfile:
        
        reader = csv.reader(infile)
        writer = csv.writer(outfile)
        
        for row in reader:
            # Check if the row starts with '0,0,0'
            if not (len(row) > 0 and row[0] == '0' and row[1] == '0' and row[2] == '0'):
                writer.writerow(row)

if __name__ == '__main__':
    filter_csv('datalog.csv', 'datalog_2.csv')
