
num_boards = 3
pins_per_board = 8
total_pins = num_boards*pins_per_board

board_ind = 0
for pp in range(total_pins):
    print(f'board: {board_ind}, pin: {pp % pins_per_board}')

    if pp % pins_per_board >= (pins_per_board-1):
        board_ind+=1

print(int(int(15) / int(pins_per_board)))
