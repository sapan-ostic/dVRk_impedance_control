
from Tkinter import *

App = Tk()
x = 0
y = 0

K_lin = 100
D_lin = 1



def init():

    global x, y
    global App
    create_gui(App)


# Def Init Function
def get_app_handle():
    global App
    return App


# Define Callbacks for Tkinter GUI Sliders
def x_cb(val):
    global x
    x = float(val)


def velocity_cb(val):
    global y
    y = float(val)


def kp_val(val):
    global K_lin
    K_lin = float(val)
#
#
def kd_val(val):
    global D_lin
    D_lin = float(val)


def create_gui(app):
    _width = 20
    _length = 300
    _resolution = 0.1
    res_val = 1
    # Define Sliders and Labels
    x_slider = Scale(app, from_=-5, to=5, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL, command=x_cb)
    x_slider.pack(expand=YES, fill=Y)
    x_label = Label(app, text="desired link 1 position value")
    x_label.pack(expand=YES, fill=Y)

    y_slider = Scale(app, from_=-5, to=5, resolution=_resolution, width=_width, length=_length, orient=HORIZONTAL, command=velocity_cb)
    y_slider.pack(expand=YES, fill=Y)
    y_label = Label(app, text="desired link 1 velocity value")
    y_label.pack(expand=YES, fill=Y)

    z_slider = Scale(app, from_=200, to=1500, resolution=res_val, width=_width, length=_length, orient=HORIZONTAL, command=kp_val)
    z_slider.pack(expand=YES, fill=Y)
    z_label = Label(app, text="Kp value")
    z_label.pack(expand=YES, fill=Y)
    #
    roll_slider = Scale(app, from_=0, to=100, resolution=res_val, width=_width, length=_length, orient=HORIZONTAL, command=kd_val)
    roll_slider.pack(expand=YES, fill=Y)
    roll_label = Label(app, text="Kd value")
    roll_label.pack(expand=YES, fill=Y)

