import matplotlib.pyplot as plt
position_error = []

def plot():
    with open("/home/freicar/freicar_ws/src/freicar-2020-exercises//02-01-localization-exercise/freicar_localization/src/position_error.txt", "r") as position:
        for value in position:
            store_pos = value.split(",")
            store_pos.pop()
    for i in store_pos:
        position_error.append(float(i))
    fig,ax = plt.subplots()
    ax.plot(range(0,len(position_error)), position_error, color ='red', label = "position error")
    plt.legend()
    plt.xlabel('Time steps')
    plt.ylabel("Position Error [m]")
    plt.title("Position Error")
    plt.show()
plot()
