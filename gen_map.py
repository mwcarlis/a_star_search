import random, copy, time



def gen_map(num_rows, num_columns, file_name):
    with open(file_name, "a") as file_d:
        for rows in xrange(num_rows):
            for columns in xrange(num_columns):
                if rows == 0 and columns == 0:
                    file_d.write('I')
                elif rows == num_rows - 1 and columns == num_columns - 1:
                    file_d.write('P')
                else:
                    file_d.write(str(random.randrange(1,10)))
            file_d.write('\n')

def write_file(matrix, file_name):
    with open(file_name, 'w') as file_d:
        #map_writer = csv.writer(file_d, delimiter='')
        for r_cnt, row in enumerate(matrix):
            for v_cnt, val in enumerate(row):
                file_d.write(str(val))
            file_d.write('\n')

    pass
if __name__ == '__main__':
    this = []
    i_time = time.time()
    print 'running'
    size = 30
    gen_map(size, size, 'max_size_3.txt')
    #write_file(this, 'my_name.carlis.txt')
    print 'done', time.time() - i_time
