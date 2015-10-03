from qsr_rcc2 import QSR_RCC2
from qsr_rcc3_rectangle_bounding_boxes_2d import QSR_RCC3_Rectangle_Bounding_Boxes_2D
from qsr_rcc4 import QSR_RCC4
from qsr_rcc5 import QSR_RCC5
from qsr_rcc8 import QSR_RCC8
from qsr_cardinal_direction import QSR_Cardinal_Direction
from qsr_qtc_b_simplified import QSR_QTC_B_Simplified
from qsr_qtc_c_simplified import QSR_QTC_C_Simplified
from qsr_qtc_bc_simplified import QSR_QTC_BC_Simplified
from qsr_arg_relations_distance import QSR_Arg_Relations_Distance
from qsr_arg_prob_relations_distance import QSR_Arg_Prob_Relations_Distance
from qsr_moving_or_stationary import QSR_Moving_or_Stationary
from qsr_new_mwe import QSR_MWE
from qsr_ra import QSR_RA
from qsr_tpcc import QSR_TPCC
# register new qsrs by class name below
qsrs_registry = (QSR_RCC2,
                 QSR_RCC3_Rectangle_Bounding_Boxes_2D,
                 QSR_RCC4,
                 QSR_RCC5,
                 QSR_RCC8,
                 QSR_Cardinal_Direction,
                 QSR_QTC_B_Simplified,
                 QSR_QTC_C_Simplified,
                 QSR_QTC_BC_Simplified,
                 QSR_Arg_Relations_Distance,
                 QSR_Arg_Prob_Relations_Distance,
                 QSR_Moving_or_Stationary,
                 QSR_MWE,
                 QSR_RA,
                 QSR_TPCC)
