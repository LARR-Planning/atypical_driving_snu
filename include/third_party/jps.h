//
// Created by jungwon on 20. 4. 23..
//

#ifndef ATYPICAL_DRIVING_JPS_H
#define ATYPICAL_DRIVING_JPS_H

#include <queue>
#include <unordered_set>

class Node{
// Variables used here are constantly accessed and checked; leaving public for now.
public:
    /** \brief x coordinate */
    int x_;
    /** \brief y coordinate */
    int y_;
    /** \brief Node id */
    int id_;
    /** \brief Node's parent's id */
    int pid_;
    /** \brief cost to reach this node */
    double cost_;
    /** \brief heuristic cost to reach the goal */
    double h_cost_;

    /**
    * @brief Constructor for Node class
    * @param x X value
    * @param y Y value
    * @param cost Cost to get to this node
    * @param h_cost Heuritic cost of this node
    * @param id Node's id
    * @param pid Node's parent's id
    */
    Node(int x = 0, int y = 0, double cost = 0, double h_cost = 0, int id = 0, int pid = 0){
        this->x_ = x;
        this->y_ = y;
        this->cost_ = cost;
        this->h_cost_ = h_cost;
        this->id_ = id;
        this->pid_ = pid;
    }

    /**
    * @brief Prints the values of the variables in the node
    * @return void
    */
    void PrintStatus(){
        std::cout << "--------------"              << std::endl
                  << "Node          :"             << std::endl
                  << "x             : " << x_      << std::endl
                  << "y             : " << y_      << std::endl
                  << "Cost          : " << cost_   << std::endl
                  << "Heuristic cost: " << h_cost_ << std::endl
                  << "Id            : " << id_     << std::endl
                  << "Pid           : " << pid_    << std::endl
                  << "--------------"              << std::endl;
    }

    /**
    * @brief Overloading operator + for Node class
    * @param p node
    * @return Node with current node's and input node p's values added
    */
    Node operator+(Node p){
        Node tmp;
        tmp.x_ = this->x_ + p.x_;
        tmp.y_ = this->y_ + p.y_;
        tmp.cost_ = this->cost_ + p.cost_;
        return tmp;
    }

    /**
    * @brief Overloading operator - for Node class
    * @param p node
    * @return Node with current node's and input node p's values subtracted
    */
    Node operator-(Node p){
        Node tmp;
        tmp.x_ = this->x_ - p.x_;
        tmp.y_ = this->y_ - p.y_;
        return tmp;
    }

    /**
    * @brief Overloading operator = for Node class
    * @param p node
    * @return void
    */
    void operator=(Node p){
        this->x_ = p.x_;
        this->y_ = p.y_;
        this->cost_ = p.cost_;
        this->h_cost_ = p.h_cost_;
        this->id_ = p.id_;
        this->pid_ = p.pid_;
    }

    /**
    * @brief Overloading operator == for Node class
    * @param p node
    * @return bool whether current node equals input node
    */
    bool operator==(Node p){
        if (this->x_ == p.x_ && this->y_ == p.y_) return true;
        return false;
    }

    /**
    * @brief Overloading operator != for Node class
    * @param p node
    * @return bool whether current node is not equal to input node
    */
    bool operator!=(Node p){
        if (this->x_ != p.x_ || this->y_ != p.y_) return true;
        return false;
    }
};

/**
* @brief Struct created to encapsulate function compare cost between 2 nodes. Used in with multiple algorithms and classes
*/
struct compare_cost{

    /**
    * @brief Compare cost between 2 nodes
    * @param p1 Node 1
    * @param p2 Node 2
    * @return Returns whether cost to get to node 1 is greater than the cost to get to node 2
    */
    bool operator()(Node& p1, Node& p2){
        // Can modify this to allow tie breaks based on heuristic cost if required
        if (p1.cost_ + p1.h_cost_ > p2.cost_ + p2.h_cost_) return true;
        else if (p1.cost_ + p1.h_cost_ == p2.cost_ + p2.h_cost_ && p1.h_cost_ >= p2.h_cost_) return true;
        return false;
    }
};

std::vector<Node> GetMotion(){
    Node down(0,1,1,0,0,0);
    Node up(0,-1,1,0,0,0);
    Node left(-1,0,1,0,0,0);
    Node right(1,0,1,0,0,0);
    std::vector<Node> v;
    v.push_back(down);
    v.push_back(up);
    v.push_back(left);
    v.push_back(right);
    // NOTE: Add diagonal movements for A* and D* only after the heuristics in the
    // algorithms have been modified. Refer to README.md. The heuristics currently
    // implemented are based on Manhattan distance and dwill not account for
    //diagonal/ any other motions
    return v;
}

class JumpPointSearch{
public:
    std::vector<Node> jump_point_search(std::vector<std::vector<int>> &grid, Node start_in, Node goal_in);
    void InsertionSort(std::vector<Node>& v);
    bool has_forced_neighbours(Node& new_point, Node& next_point, Node& motion);
    Node jump(Node& new_point, Node& motion, int id);
private:
    std::priority_queue<Node, std::vector<Node>, compare_cost> open_list_;
    std::vector<std::vector<int>> grid;
    std::vector<Node> closed_list_;
    std::unordered_set<int> pruned;
    Node start_, goal_;
    int n;
};

Node JumpPointSearch::jump(Node& new_point, Node& motion, int id){
    Node next_point  = new_point + motion;
    next_point.id_ = n*next_point.x_+next_point.y_;
    next_point.pid_ = id;
    next_point.h_cost_ = abs(next_point.x_ - goal_.x_) + abs(next_point.y_ - goal_.y_);
    if(next_point.x_ < 0 || next_point.y_ < 0 || next_point.x_ >= n || next_point.y_ >= n || grid[next_point.x_][next_point.y_]!=0){
        return new_point;
        // return Node(-1,-1,-1,-1,-1,-1);
    }
    if(pruned.find(next_point.id_)!=pruned.end()) pruned.insert(next_point.id_);
    if(next_point == goal_) return next_point;
    bool fn = false;
    fn = has_forced_neighbours(new_point, next_point, motion);
    if(fn){
        // std::cout << "Forced neighbours found"<<std::endl;
        return next_point;
    }
    else{
        Node jump_node = jump(next_point, motion, id);
        // Prevent over shoot
        if(jump_node.cost_ !=-1 &&  jump_node.cost_+ jump_node.h_cost_  <= next_point.cost_ + next_point.h_cost_) return jump_node;
        else return next_point;
    }
}

bool JumpPointSearch::has_forced_neighbours(Node& new_point, Node& next_point, Node& motion){
    int cn1x = new_point.x_ + motion.y_;
    int cn1y = new_point.y_ + motion.x_;

    int cn2x = new_point.x_ - motion.y_;
    int cn2y = new_point.y_ - motion.x_;

    int nn1x = next_point.x_ + motion.y_;
    int nn1y = next_point.y_ + motion.x_;

    int nn2x = next_point.x_ - motion.y_;
    int nn2y = next_point.y_ - motion.x_;

    bool a = !(cn1x < 0 || cn1y < 0 || cn1x >= n || cn1y >= n || grid[cn1x][cn1y]==1);
    bool b = !(nn1x < 0 || nn1y < 0 || nn1x >= n || nn1y >= n || grid[nn1x][nn1y]==1);
    if(a!=b) return true;

    a = !(cn2x < 0 || cn2y < 0 || cn2x >= n || cn2y >= n || grid[cn2x][cn2y]==1);
    b = !(nn2x < 0 || nn2y < 0 || nn2x >= n || nn2y >= n || grid[nn2x][nn2y]==1);
    if(a!=b) return true;

    return false;

}

#ifdef CUSTOM_DEBUG_HELPER_FUNCION
void JumpPointSearch::InsertionSort(std::vector<Node>& v){
   int nV = v.size();
   int i, j;
   Node key;
   for (i = 1; i < nV; i++) {
       key = v[i];
       j = i-1;
       while (j >= 0 && (v[j].cost_ + v[j].h_cost_ > key.cost_+key.h_cost_)){
           v[j+1] = v[j];
           j--;
       }
       v[j+1] = key;
   }
}
#endif

std::vector<Node> JumpPointSearch::jump_point_search(std::vector<std::vector<int>> &grid, Node start_in, Node goal_in){
    this->grid = grid;
    start_ = start_in;
    goal_ = goal_in;
    n = grid.size();
    // Get possible motions
    std::vector<Node> motion = GetMotion();
    open_list_.push(start_);

    // Main loop
    Node temp;
    while(!open_list_.empty()){
        Node current = open_list_.top();
        open_list_.pop();
        current.id_ = current.x_ * n + current.y_;
        if(current.x_ == goal_.x_ && current.y_ == goal_.y_){
            closed_list_.push_back(current);
            grid[current.x_][current.y_] = 2;
            return closed_list_;
        }
        grid[current.x_][current.y_] = 2; // Point opened
        int current_cost = current.cost_;
        for(auto it = motion.begin(); it!=motion.end(); ++it){
            Node new_point;
            new_point = current + *it;
            new_point.id_ = n*new_point.x_+new_point.y_;
            new_point.pid_ = current.id_;
            new_point.h_cost_ = abs(new_point.x_ - goal_.x_) + abs(new_point.y_ - goal_.y_);
            if(new_point == goal_){
                open_list_.push(new_point);
                break;
            }
            if(new_point.x_ < 0 || new_point.y_ < 0 || new_point.x_ >= n || new_point.y_ >= n) continue; // Check boundaries
            if(grid[new_point.x_][new_point.y_]!=0){
                continue; //obstacle or visited
            }

            Node jump_point = jump(new_point, *it, current.id_);
            if(jump_point.id_!=-1){
                open_list_.push(jump_point);
                if(jump_point.x_ == goal_.x_ && jump_point.y_ == goal_.y_){
                    closed_list_.push_back(current);
                    closed_list_.push_back(jump_point);
                    grid[jump_point.x_][jump_point.y_] = 2;
                    return closed_list_;
                }
            }
            open_list_.push(new_point);
        }
        closed_list_.push_back(current);
    }
    closed_list_.clear();
    Node no_path_node(-1,-1,-1,-1,-1,-1);
    closed_list_.push_back(no_path_node);
    return closed_list_;
}

#endif //ATYPICAL_DRIVING_JPS_H
