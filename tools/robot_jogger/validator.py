# -*- coding: utf-8 -*-
"""
Created on Wed Apr 17 09:31:39 2019

@author: John
"""

import re
import ipaddress

from PyQt5.QtGui import QValidator, QIntValidator, QDoubleValidator


def isValidIPAddress(ip):
    try:
        ipaddress.ip_address(ip)
    except ValueError:
        return False
    else:
        return True
    

class DoubleValidator(QDoubleValidator):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def validate(self, input_, pos):
        state, input_, pos = super().validate(input_, pos)
        if not input_ or input_ == ".":
            return QValidator.Intermediate, input_, pos
        if state != QValidator.Acceptable:
            return QValidator.Invalid, input_, pos
        return QValidator.Acceptable, input_, pos

    def fixup(self, input_):
        pass


class IPAddressValidator(QValidator):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def validate(self, input_, pos):
        if re.match(r"[0-9,.]*$", input_) \
            and input_.count(".") <= 3 and len(input_) <= 15:
            return QValidator.Intermediate, input_, pos        
        elif isValidIPAddress(input_):
            return QValidator.Acceptable, input_, pos
        else:
            return QValidator.Invalid, input_, pos  

    def fixup(self, input_):
        pass


class PortNumberValidator(QIntValidator):
    def __init__(self, *args, **kwargs):
        super().__init__(bottom=1, top=65535, *args, **kwargs)

    def validate(self, input_, pos):
        if input_ == "0":
            return QValidator.Invalid, input_, pos
        else:
            return super().validate(input_, pos)

    def fixup(self, input_):
        pass