from _frankx import Robot as _Robot
from .gripper import Gripper as _Gripper


class Robot(_Robot):
    logged_in = False

    def get_gripper(self):
        return _Gripper(self.fci_ip)

    def login(self, name, password):
        from selenium import webdriver
        from selenium.webdriver.firefox.options import Options
        from selenium.webdriver.common.keys import Keys

        options = Options()
        # options.headless = True

        driver = webdriver.Firefox(options=options)
        driver.get(f'https://{self.fci_ip}')

        elem = driver.find_element_by_xpath('//input[@id=(//label[text()="Username"]/@for)]')
        elem.clear()
        elem.send_keys(name)

        elem = driver.find_element_by_xpath('//input[@id=(//label[text()="Password"]/@for)]')
        elem.clear()
        elem.send_keys(password)
        elem.send_keys(Keys.RETURN)

        self.logged_in = True

    def unlock_brakes(self):
        if not self.logged_in:
            return

        # elem = driver.find_element_by_xpath('//div[@data-bind]')

    def lock_brakes(self):
        if not self.logged_in:
            return
