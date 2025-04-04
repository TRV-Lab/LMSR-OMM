import math
#gt_stored_roads={}
#stored_roads={}
# 地球半径，单位：米
EARTH_RADIUS = 6378137


def haversine_distance(lat1, lon1, lat2, lon2):
    """
    该函数用于计算两个经纬度坐标点之间的球面距离
    :param lat1: 第一个点的纬度
    :param lon1: 第一个点的经度
    :param lat2: 第二个点的纬度
    :param lon2: 第二个点的经度
    :return: 两点之间的距离，单位为米
    """
    # 将角度转换为弧度
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    # 计算纬度和经度的差值
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    # Haversine 公式的中间计算部分
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    # 计算距离
    distance = EARTH_RADIUS * c
    return distance

def calculate_polyline_length(coordinates):
    """
    该函数用于计算一系列经纬度坐标点连接成的折线的总长度
    :param coordinates: 包含经纬度坐标的列表，每个元素是一个 (纬度, 经度) 元组
    :return: 折线的总长度，单位为米
    """
    total_length = 0
    for i in range(len(coordinates) - 1):
        # 获取相邻的两个点
        lat1, lon1 = coordinates[i]
        lat2, lon2 = coordinates[i + 1]
        # 计算两点之间的距离并累加到总长度中
        total_length += haversine_distance(lat1, lon1, lat2, lon2)
        #print(i,total_length)
    return total_length


def process_files(gt_path, result_path):
    gt = []
    result = []

    # 读取第一个文件的内容
    with open(gt_path, 'r') as f1:
        for line in f1.readlines():
            columns = line.strip().split(',')
            gt.append((columns[0], columns[1],float(columns[2]),float(columns[6]), float(columns[7])))#6/3是纬度，7/4是精度
        # print(gt)
    # 读取第二个文件的内容
    with open(result_path, 'r') as f2:
        for line in f2.readlines():
            columns = line.strip().split(',')
            result.append((columns[0], columns[1],float(columns[2]),float(columns[6]), float(columns[7])))#6/3是纬度，7/4是精度
        # print(result)
    
    N_all=len(result)-1
    print("N_all",N_all)
    N_correct=N_all
    P_gt=0
    P_result=0
    P_intersection_gps=0
    P_intersection_road=0
    #for row in range(len(gt)-1):
        #if (gt[row][0], gt[row][1]) not in gt_stored_roads:
            #P_gt+=gt[row][4]#用了gt文件道路长度总和
            #gt_stored_roads[(gt[row][0], gt[row][1])]=True
    #GT长度
    extracted_coordinates = []
    for coord in gt:
        # 提取前两列元素
        extracted_coordinates.append((coord[3], coord[4]))
    P_gt=calculate_polyline_length(extracted_coordinates)
    print("P_gt",P_gt)
    #匹配长度
    extracted_coordinates = []
    for coord in result:
        # 提取前两列元素
        extracted_coordinates.append((coord[3], coord[4]))
    P_result=calculate_polyline_length(extracted_coordinates)
    print("P_result",P_result)
    P_correct=0
    gt_set = set((item[0], item[1]) for item in gt)
    result_set = set((item[0], item[1]) for item in result)
    for row in range(len(result)):
        if (result[row][0], result[row][1]) not in gt_set:
            N_correct-=1
            print("wrong from",result[row][2],haversine_distance(result[row-1][3], result[row-1][4], result[row][3], result[row][4]))
        elif (row>0 and (result[row-1][0], result[row-1][1]) in gt_set):
            P_correct+=haversine_distance(result[row-1][3], result[row-1][4], result[row][3], result[row][4])
    print("P_correct",P_correct)                        

    #计算集合差集
    missing_elements=gt_set-result_set
    print("missing_elements",missing_elements)
    
    match_rate=N_correct/N_all
    print("N_correct",N_correct)

    precision=P_correct/P_result
    recall=P_correct/P_gt

    F1=2*precision*recall/(precision+recall)
    
    return (match_rate,precision,recall,F1)

# 指定两个文件的路径
id=6
#gt_path ='./se_results/gt_se#{}.txt'.format(id)
#result_path ='./se_results/both_ablation_se#{}.txt'.format(id)
#result_path = './se_results/holistic_path_se#{}.txt'.format(id)
#result_path = './se_results/lane_marking_map_se#{}.txt'.format(id)
#result_path = './se_results/P2C_se#{}.txt'.format(id)
#result_path = './se_results/hmm_se#{}.txt'.format(id)
#result_path = './se_results/HMM_OBDSC_se#{}.txt'.format(id)
#result_path = './se_results/proposed_GNSS_se#{}.txt'.format(id)
#result_path = './se_results/proposed_PSD_se#{}.txt'.format(id)
#result_path ='./se_results/AMM_se#{}.txt'.format(id)
#result_path ='./se_results/OBDSC_se#{}.txt'.format(id)
#result_path ='./se_results/PSD_P2C_se#{}.txt'.format(id)
#result_path ='./se_results/OBDSC_lane_marking_se#{}.txt'.format(id)
#result_path = './se_results/OXTS_se#{}.txt'.format(id)
#result_path = './se_results/new_PSD_P2C_se#{}.txt'.format(id)


gt_path = './shanghai_results/gt_shanghai#{}.txt'.format(id)
#result_path ='./shanghai_results/P2C_GNSS_shanghai#{}.txt'.format(id)
#result_path ='./shanghai_results/P2C_PSD_shanghai#{}.txt'.format(id)
#result_path = './shanghai_results/P2C_OXTS_shanghai#{}.txt'.format(id)
#result_path ='./shanghai_results/HMM_shanghai#{}.txt'.format(id)
#result_path = './shanghai_results/AMM_shanghai#{}.txt'.format(id)
#result_path ='./shanghai_results/HMM_OBDSC_shanghai#{}.txt'.format(id)
#result_path ='./shanghai_results/both_ablation_shanghai#{}.txt'.format(id)
#result_path ='./shanghai_results/OBDSC_shanghai#{}.txt'.format(id)
#result_path ='./shanghai_results/lane_marking_map_shanghai#{}.txt'.format(id)
result_path ='./shanghai_results/degrade_lane_marking_map_shanghai#{}.txt'.format(id)
#result_path = './shanghai_results/OBDSC_lane_marking_shanghai#{}.txt'.format(id)
#result_path = './shanghai_results/proposed_GNSS_shanghai#{}.txt'.format(id)
#result_path = './shanghai_results/proposed_PSD_shanghai#{}.txt'.format(id)
#result_path ='./shanghai_results/holistic_path_shanghai#{}.txt'.format(id)
#result_path ='./shanghai_results/lane_marker_map_shanghai#{}.txt'.format(id)

(match_rate,precision,recall,F1) = process_files(gt_path, result_path)
print("结果为: {}".format((match_rate,precision,recall,F1)))

