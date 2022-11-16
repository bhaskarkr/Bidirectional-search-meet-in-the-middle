from scipy.stats import f_oneway

class ANOVATest:
    def __init__(self, alpha=0.05):
        self.alpha = alpha

    def conduct_test(group_1, group_2):
        F_score, p_value = f_oneway(performance1, performance2, performance3, performance4)
        
        if p_value > self.alpha:
            print("P-value of", p_value, "is > alpha", self.alpha, "rejecting null-hypothesis that Var(group_1) == Var(group_2)")
        else:
            print("P-value of", p_value, "is <= alpha", self.alpha, "proving null-hypothesis that Var(group_1) == Var(group_2)")