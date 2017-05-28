#include<vector>
#include<iostream>

#include"Eigen/Dense"
using namespace Eigen;
class OptimalController{
static int n=9;
static int m=3;
public:
    OptimalController();
    ~OptimalController();  
    int init(std::string filename);    
    int get_control(MatrixXd state_now, double tgo, MatrixXd &u);
private:
    vector<vector<double>> _plist;    
    vector<double> _ptspace;        
    int _ptm;
    int _ptn;    
    //R^-1*B'*P
    MatrixXd<n,m> _B;        
    MatrixXd<m,m> _R;    
    MatrixXd<m,1> _umax;
    vector<MatrixXd(n,n)> _pl;        
    std::string _filename;    
};
