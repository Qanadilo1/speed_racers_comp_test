import matplotlib.pyplot as plt
# Made by Davide Rezzoli
mIoU_all = []
l1_all = []
mIoU_all_validation = []
l1_all_validation = []


def plot():
    with open("training_miou.txt", "r") as mIoU:
        for value in mIoU:
            store_mIoU = value.split(",")
            store_mIoU.pop()
    for i in store_mIoU:
        mIoU_all.append(float(i))
    with open("evaluation_miou.txt", "r") as mIoU_v:
        for value in mIoU_v:
            store_miou_validation = value.split(",")
            store_miou_validation.pop()
    for i in store_miou_validation:
        mIoU_all_validation.append(float(i))
        mIoU_all_validation.append(None)
        mIoU_all_validation.append(None)
    with open("training_L1.txt", "r") as l1:
        for value in l1:
            store_l1 = value.split(",")
            store_l1.pop()
    for i in store_l1:
        l1_all.append(float(i))
    with open("evaluation_L1.txt", "r") as l1_val:
        for value in l1_val:
            store_l1_validation = value.split(",")
            store_l1_validation.pop()
    for i in store_l1_validation:
        l1_all_validation.append(float(i))
        l1_all_validation.append(None)
        l1_all_validation.append(None)
    fig, ax = plt.subplots()
    ax.plot(range(0, len(mIoU_all)), mIoU_all, color='red', label="training")
    ax.plot(range(0, len(mIoU_all_validation)), mIoU_all_validation, "bo", label="evaluation")
    plt.legend()
    plt.xlabel('epochs')
    plt.ylabel("mIoU")
    plt.title("comparison of differnt mIoU")
    plt.show()
    fig, ax = plt.subplots()
    ax.plot(range(0, len(l1_all)), l1_all, color='red', label="training")
    ax.plot(range(0, len(l1_all_validation)), l1_all_validation, "bo", label="evaluation")
    plt.legend()
    plt.xlabel('epochs')
    plt.ylabel("l1 regression loss")
    plt.title("comparison of different line regression loss")
    plt.show()


plot()