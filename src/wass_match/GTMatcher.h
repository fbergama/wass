#ifndef _GT_MATCHER_H_
#define _GT_MATCHER_H_


#include "FeatureSet.h"
#include <vector>


namespace WASS
{
namespace match
{

struct Match
{
public:
    size_t first;
    size_t second;
    cv::Vec2f imga_loc;
    cv::Vec2f imgb_loc;
};

typedef std::vector< Match > MatchList;





class GTMatcherImpl;
class GTMatcher
{
public:
    class GTMatcherException : public std::exception
    {
    public:
        inline GTMatcherException( std::string _wh ) : wh(_wh) {}
        virtual const char* what() const noexcept { return wh.c_str(); }

    private:
        std::string wh;
    };

    explicit GTMatcher( FeatureSet& _fa, FeatureSet& _fb  );
    ~GTMatcher();

    MatchList generate_candidates( );
    void compute_payoff_matrix( double payoff_lambda_geometric );
    MatchList match_group( double pop_threshold );

    int n_candidates_per_feature;

    static void save_matches( std::string filename, const MatchList& matches );
    static void load_matches( std::string filename, MatchList& matches );

private:
    GTMatcherImpl* pImpl;
};

}
}

#endif
