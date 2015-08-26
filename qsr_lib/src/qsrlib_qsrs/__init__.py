from qsr_rcc2_rectangle_bounding_boxes_2d import QSR_RCC2_Rectangle_Bounding_Boxes_2D
from qsr_rcc3_rectangle_bounding_boxes_2d import QSR_RCC3_Rectangle_Bounding_Boxes_2D
from qsr_rcc8_rectangle_bounding_boxes_2d import QSR_RCC8_Rectangle_Bounding_Boxes_2D
from qsr_cone_direction_bounding_boxes_centroid_2d import QSR_Cone_Direction_Bounding_Boxes_Centroid_2D
from qsr_qtc_b_simplified import QSR_QTC_B_Simplified
from qsr_qtc_c_simplified import QSR_QTC_C_Simplified
from qsr_qtc_bc_simplified import QSR_QTC_BC_Simplified
from qsr_arg_relations_distance import QSR_Arg_Relations_Distance
from qsr_arg_prob_relations_distance import QSR_Arg_Prob_Relations_Distance
from qsr_moving_or_stationary import QSR_Moving_or_Stationary
from qsr_new_mwe import QSR_MWE

# register new qsrs by class name below
qsrs_registry = (QSR_RCC2_Rectangle_Bounding_Boxes_2D,
                 QSR_RCC3_Rectangle_Bounding_Boxes_2D,
                 QSR_RCC8_Rectangle_Bounding_Boxes_2D,
                 QSR_Cone_Direction_Bounding_Boxes_Centroid_2D,
                 QSR_QTC_B_Simplified,
                 QSR_QTC_C_Simplified,
                 QSR_QTC_BC_Simplified,
                 QSR_Arg_Relations_Distance,
                 QSR_Arg_Prob_Relations_Distance,
                 QSR_Moving_or_Stationary,
                 QSR_MWE)
