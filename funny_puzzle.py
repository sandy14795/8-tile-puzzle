import heapq
import numpy as np
import copy 

goal_state = [1,2,3,4,5,6,7,8,0]
#goal_state = [1,2,3,8,0,4,7,6,5]
pq = []

goal_state_cds = {1 : [0,0], 2: [0,1], 3: [0,2], 4: [1,0] , 5: [1,1], 6: [1,2], 7:[2,0], 8:[2,1], 0:[2,2]}

succ_state_hs = []

closed_states = []
visited = set()
state_path = {}

def h_s(state):
    st = np.array(state).reshape(3,3)
    sum = 0
    for i in range(3):
        for j in range(3):
            if(st[i][j]) != 0:
                sum += abs(i - goal_state_cds[st[i][j]][0]) + abs(j - goal_state_cds[st[i][j]][1])
    return sum

def print_succ(state, log = True):
    global succ_state_hs
    succ_state = []
    st = np.array(state).reshape(3,3)
    #print(st)
    init_state = copy.deepcopy(st)
    t = np.where(st == 0)
    i,j = t[0][0],t[1][0]
    four_states = [[1,0], [-1,0], [0,1], [0,-1]]
    
    for k in four_states:
        swap_i = i+k[0]
        swap_j = j+k[1]
        #print(st)
        if(swap_i >= 0 and swap_i < 3) and (swap_j >=0 and swap_j < 3):
            st[i][j] = st[swap_i][swap_j]
            st[swap_i][swap_j] = 0
            
            succ_state.append(st.reshape(1,-1).tolist()[0])
        
            st = copy.deepcopy(init_state)
            
    succ_state.sort()
    succ_state_hs = []
    for sux_state in succ_state:
        hs = h_s(sux_state)
        if(log):
            print(str(sux_state) + " h="+str(hs))
        succ_state_hs.append([sux_state, hs])

def print_path():
    res = []
    res.append([tuple(goal_state), 0])
    p = state_path[tuple(goal_state)]
    while(p[0] != (-1,)):
        res.append(p)
        p = state_path[p[0]]
        
    moves = 0
    for i in range(len(res)-1,-1,-1):
        print(str(list(res[i][0])) + " h=" + str(res[i][1]) + " moves: " + str(moves))
        moves += 1
     
def solve(state, g_s = 0, parent = -1):
    global succ_state_hs, state_path
    hs = h_s(state)
    heapq.heappush(pq, ((hs+g_s), state, (g_s, hs, parent)) )
    visited.add(tuple(state))
    state_path[tuple(state)] = [tuple([parent]) , hs]
    while(pq != []):
        curr_st = heapq.heappop(pq)
        #print(str(curr_st[1]) + " h=" + str(curr_st[2][1]) + " g=" + str(curr_st[2][0]))
        closed_states.append(curr_st)
        
        if curr_st[1] == goal_state:
            break
        
        print_succ(curr_st[1], log = False)
        children = succ_state_hs
        
        for child in children:
            
            if tuple(child[0]) in visited:   
                
                for pq_st in pq:
                    if pq_st[1] == child[0] and (curr_st[2][0]+1) < pq_st[2][0]:
                        pq.remove(pq_st)
                        heapq.heapify(pq)
                        heapq.heappush(pq, (curr_st[2][0]+1 + child[1], child[0], (curr_st[2][0]+1, child[1],curr_st[2][2]+1)))
                        state_path[tuple(child[0])] = [tuple(curr_st[1]), curr_st[2][1]]
        
                for i in range(len(closed_states)):
                    if closed_states[i][1] == child[0] and (curr_st[2][0]+1) < closed_states[i][2][0]:
                        del closed_states[i]
                        
                        heapq.heappush(pq, (curr_st[2][0]+1 + child[1], child[0], (curr_st[2][0]+1, child[1],curr_st[2][2]+1)))
                        state_path[tuple(child[0])] = [tuple(curr_st[1]), curr_st[2][1]]
                
            else:
                heapq.heappush(pq, (curr_st[2][0]+1 + child[1], child[0], (curr_st[2][0]+1, child[1],curr_st[2][2]+1)))
                visited.add(tuple(child[0]))
                state_path[tuple(child[0])] = [tuple(curr_st[1]), curr_st[2][1]]
    
    print_path()
            
#solve([4,3,8,5,1,6,7,2,0])

#solve([2,3,1,8,0,4,7,6,5])
    
#print_succ([8,7,6,5,4,3,2,1,0])

