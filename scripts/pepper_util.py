import numpy as np
from PIL import Image, ImageDraw
from scipy.integrate import quad
from scipy.optimize import curve_fit
from skimage.morphology import medial_axis
from typing import List, Optional


class Curve:

    def __init__(self, direction: str = None, curve_x: np.array = None, curve_y: np.array = None, params: np.array = None):
        self._peduncle_direction = direction
        self._curve_x = curve_x
        self._curve_y = curve_y
        self._params = params

    @property
    def peduncle_direction(self):
        return self._peduncle_direction

    @peduncle_direction.setter
    def peduncle_direction(self, direction):
        self._peduncle_direction = direction

    @property
    def curve_x(self):
        return self._curve_x

    @curve_x.setter
    def curve_x(self, value):
        self._curve_x = value

    @property
    def curve_y(self):
        return self._curve_y

    @curve_y.setter
    def curve_y(self, value):
        self._curve_y = value

    @property
    def params(self):
        return self._params

    @params.setter
    def params(self, params):
        self._params = params


    def parabola(self, t, a, b, c):
        return a * t ** 2 + b * t + c
    
    
    def curve_length_derivative(self, t):
        return (1 + (2*self._params[0]*t + self._params[1])**2)**0.5
    

    def curve_length(self, idx):
        if self._peduncle_direction == 'horizontal':
            return abs(quad(self.curve_length_derivative, self._curve_y[0], self._curve_y[idx])[0])
        else:
            return abs(quad(self.curve_length_derivative, self._curve_x[0], self._curve_x[idx])[0])
        

    def full_curve_length(self):
        if self._peduncle_direction == 'horizontal':
            return abs(quad(self.curve_length_derivative, self._curve_y[0], self._curve_y[-1])[0])
        else:
            return abs(quad(self.curve_length_derivative, self._curve_x[0], self._curve_x[-1])[0])

    
    def fit_curve_to_mask(self, mask, fruit_xywh, peduncle_xywh):

        medial_img, _ = medial_axis(mask, return_distance=True)

        x, y = np.where(medial_img == 1)
        
        # Cannot call curve_fit if number of datapoints are less than 3
        if x.shape[0] < 3:
            return 0

        params1, _ = curve_fit(self.parabola, y, x)
        a, b, c = params1
        fit_curve_x = self.parabola(y, a, b, c)
        
        params2, _ = curve_fit(self.parabola, x, y)
        a, b, c = params2
        fit_curve_y = self.parabola(x, a, b, c)

        if np.linalg.norm(x - fit_curve_x) < np.linalg.norm(y - fit_curve_y):
            self._peduncle_direction = 'horizontal'
            self._params = params1

            # xywh: x: along rows, y: along columns
            if fruit_xywh[1] < peduncle_xywh[1]:
                # Sorted assuming that the pepper to the left of the peduncle
                # Sorting with respect to y in ascending order
                sorted_x = np.array([x for _, x in sorted(zip(y, x))])
                sorted_y = np.array([y for y, _ in sorted(zip(y, x))])
            else:
                # Sorted assuming that the pepper to the right of the peduncle
                # Sorting with respect to y in descending order
                sorted_x = np.flip(np.array([x for _, x in sorted(zip(y, x))]))
                sorted_y = np.flip(np.array([y for y, _ in sorted(zip(y, x))]))
            
            self._curve_y = sorted_y
            self._curve_x = sorted_x
            # self._curve_x = self.parabola(sorted_y, self._params[0], self._params[1], self._params[2])
        else:
            self._peduncle_direction = 'vertical'
            self._params = params2
        
            # xywh: x: along rows, y: along columns
            if fruit_xywh[0] < peduncle_xywh[0]:
                # Sorted assuming that the pepper is above the peduncle
                # Sorted with respect to x in ascending order
                sorted_x = np.array([x for x, _ in sorted(zip(x, y))])
                sorted_y = np.array([y for _, y in sorted(zip(x, y))])
            else:
                # Sorted assuming that the pepper is below the peduncle
                # Sorted with respect to x in descending order
                sorted_x = np.flip(np.array([x for x, _ in sorted(zip(x, y))]))
                sorted_y = np.flip(np.array([y for _, y in sorted(zip(x, y))]))
                    
            self._curve_x = sorted_x
            self._curve_y = sorted_y
            # self._curve_y = self.parabola(sorted_x, self._params[0], self._params[1], self._params[2])

        return 1


class PepperPeduncle:

    def __init__(self, number: int, mask=None, conf=None, percentage=0.5, segment=None, xywh=None):
        self._number: int = number
        self._mask = mask
        self._conf: float = conf
        self._percentage = percentage
        self._xywh = xywh
        self._curve = Curve()
        self._poi_px = None
        self._next_point_px = None
        self.segment = segment
        self._xyz_rs = None
        self._xyz_base = None
        self._orientation_base = None

    @property
    def number(self):
        return self._number

    @property
    def mask(self):
        return self._mask

    @mask.setter
    def mask(self, mask):
        self._mask = mask

    @property
    def conf(self):
        if self._conf is None:
            return 0
        return self._conf

    @conf.setter
    def conf(self, conf):
        self._conf = conf

    @property
    def xywh(self):
        return self._xywh

    @xywh.setter
    def xywh(self, value):
        self._xywh = value

    @property
    def curve(self):
        return self._curve

    @curve.setter
    def curve(self, curve):
        self._curve = curve

    @property
    def poi_px(self):
        return self._poi_px

    @poi_px.setter
    def poi_px(self, poi_px):
        self._poi_px = poi_px

    @property
    def next_point_px(self):
        return self._next_point_px

    @next_point_px.setter
    def next_point_px(self, next_point_px):
        self._next_point_px = next_point_px

    @property
    def xyz_rs(self):
        return self._xyz_rs
    
    @xyz_rs.setter
    def xyz_rs(self, xyz_rs):
        self._xyz_rs = xyz_rs

    @property
    def xyz_base(self):
        return self._xyz_base
    
    @xyz_base.setter
    def xyz_base(self, xyz_base):
        self._xyz_base = xyz_base

    @property
    def orientation_base(self):
        return self._orientation_base
    
    @orientation_base.setter
    def orientation_base(self, orientation_base):
        self._orientation_base = orientation_base

    def determine_poi(self, total_curve_length):
        for idx in range(len(self._curve.curve_y)):
            curve_length = self._curve.curve_length(idx)
            if abs(curve_length - self._percentage * total_curve_length) < 2:
                return idx
    
        return len(self._curve.curve_y)//2


    def set_point_of_interaction(self, fruit_xywh):
        curve_available = self._curve.fit_curve_to_mask(self._mask, fruit_xywh, self._xywh)

        if not curve_available:
            return (-1, -1), (-1, -1)
        
        total_curve_length = self._curve.full_curve_length()

        idx = self.determine_poi(total_curve_length)
        self._poi_px = (self._curve.curve_x[idx], self._curve.curve_y[idx])

        IDX = 8
        next_idx = idx + IDX if idx + IDX < len(self._curve.curve_x) else len(self._curve.curve_x) - 1
        self._next_point_px = (self._curve.curve_x[next_idx], self._curve.curve_y[next_idx])

        return self._poi_px, self._next_point_px



class PepperFruit:
    def __init__(self, number:int, xywh=None, conf=0.0, segment=None, mask=None):
        self._number: int = number

        self._xywh: Optional[List[float]] = xywh # TODO: change to xyxy?
        self._conf: float = conf
        self._xyz_rs = None
        self._xyz_base = None
        self._mask = mask
        self.segment = segment

    @property
    def number(self):
        return self._number

    @property
    def xywh(self):
        return self._xywh

    @xywh.setter
    def xywh(self, xywh):
        self._xywh = xywh

    @property
    def conf(self):
        return self._conf

    @conf.setter
    def conf(self, conf):
        self._conf = conf

    @property
    def xyz_rs(self):
        return self._xyz_rs
    
    @xyz_rs.setter
    def xyz_rs(self, xyz_rs):
        self._xyz_rs = xyz_rs

    @property
    def xyz_base(self):
        return self._xyz_base
    
    @xyz_base.setter
    def xyz_base(self, xyz_base):
        self._xyz_base = xyz_base

    @property
    def mask(self):
        return self._mask
    
    @mask.setter
    def mask(self, mask):
        self._mask = mask

    def __str__(self):
        return f"Pepper(number={self.number}, xywh={self.xywh}, conf={self._conf})"
    

class Pepper:
    def __init__(self, number: int, pf_number: int, pp_number: int):
        self._number = number
        self._order: int = -1
        self._pepper_fruit: PepperFruit = PepperFruit(pf_number)
        self._pepper_peduncle: PepperPeduncle = PepperPeduncle(pp_number)

    @property
    def number(self):
        return self._number

    @property
    def order(self):
        return self._order

    @order.setter
    def order(self, order):
        self._order = order

    @property
    def pepper_fruit(self):
        return self._pepper_fruit

    @pepper_fruit.setter
    def pepper_fruit(self, value):
        self._pepper_fruit = value

    @property
    def pepper_peduncle(self):
        return self._pepper_peduncle

    @pepper_peduncle.setter
    def pepper_peduncle(self, value):
        self._pepper_peduncle = value

    def __str__(self):
        return f"Pepper #{self.number}"

