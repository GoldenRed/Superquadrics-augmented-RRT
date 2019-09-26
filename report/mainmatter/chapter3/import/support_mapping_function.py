def support_mapping_function(points, direction, reverse=False):
    if reverse==True: 
        direction = -1*direction
    bestIndex = 0
    bestDotP = np.dot((points[0][bestIndex], points[1][bestIndex], points[2][bestIndex]), direction)
    for i in range(len(points)):
        dotProd = np.dot([points[0][i], points[1][i], points[2][i]], direction)
        if dotProd>bestDotP:
            bestIndex = i
            bestDotP = d
    furthestPoint = np.array([points[0][bestIndex], points[1][bestIndex], points[2][bestIndex]])
    return furthestPoint