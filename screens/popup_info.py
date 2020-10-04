import kivy

from kivy.lang import Builder
from kivy.uix.gridlayout import GridLayout
from kivy.uix.popup import Popup
from kivy.properties import StringProperty  # @UnresolvedImport
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.widget import Widget
from kivy.uix.textinput import TextInput
from kivy.uix.label import Label
from kivy.uix.button import  Button
from kivy.uix.image import Image
from kivy.clock import Clock
from kivy.uix.checkbox import CheckBox
from kivy.graphics import Color, Rectangle

class PopupUSBInfo(Widget):

	def __init__(self, screen_manager, safe_to_remove):

		self.sm = screen_manager

		if safe_to_remove == 'mounted':
			description = 'USB stick found!\n\nPlease don\'t remove your USB stick until it is safe to do so.'

			ok_button = Button(text='[b]Ok[/b]', markup = True)
			ok_button.background_normal = ''
			ok_button.background_color = [230 / 255., 74 / 255., 25 / 255., 1.]

		elif safe_to_remove == False:
			description = 'Don\'t remove your USB stick yet.\n\nPlease wait...'

			ok_button = Button(text='[b]Ok[/b]', markup = True)
			ok_button.background_normal = ''
			ok_button.background_color = [230 / 255., 74 / 255., 25 / 255., 1.]

		elif safe_to_remove == True:
			description = 'It is now safe to remove your USB stick.'          
			ok_button = Button(text='[b]Ok[/b]', markup = True)
			ok_button.background_normal = ''
			ok_button.background_color = [76 / 255., 175 / 255., 80 / 255., 1.]

		img = Image(source="./asmcnc/apps/shapeCutter_app/img/error_icon.png", allow_stretch=False)
		label = Label(size_hint_y=1, text_size=(360, None), halign='center', valign='middle', text=description, color=[0,0,0,1], padding=[40,20], markup = True)


  
		btn_layout = BoxLayout(orientation='horizontal', spacing=10, padding=[0,0,0,0])
#         btn_layout.add_widget(back_button)
		btn_layout.add_widget(ok_button)
	
		layout_plan = BoxLayout(orientation='vertical', spacing=10, padding=[40,20,40,20])
		layout_plan.add_widget(img)
		layout_plan.add_widget(label)
		layout_plan.add_widget(btn_layout)

		self.popup = Popup(title='Warning!',
					  title_color=[0, 0, 0, 1],
					  title_font= 'Roboto-Bold',
					  title_size = '20sp',
					  content=layout_plan,
					  size_hint=(None, None),
					  size=(300, 300),
					  auto_dismiss= False
					  )

		self.popup.separator_color = [230 / 255., 74 / 255., 25 / 255., 1.]
		self.popup.separator_height = '4dp'
		self.popup.background = './asmcnc/apps/shapeCutter_app/img/popup_background.png'

		ok_button.bind(on_press=self.popup.dismiss)
#         back_button.bind(on_press=popup.dismiss)       

		self.popup.open()  