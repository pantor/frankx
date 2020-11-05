from time import sleep
from threading import Thread

from _frankx import Robot as _Robot
from .gripper import Gripper as _Gripper


class Robot(_Robot):
    logged_in = False

    def __init__(self, fci_ip, username=None, password=None):
        super().__init__(fci_ip)
        self.username = username
        self.password = password
        self.driver = None

    def move_async(self, *args) -> Thread:
        p = Thread(target=self.move, args=tuple(args), daemon=True)
        p.start()
        sleep(0.001)  # Sleep one control cycle
        return p

    def get_gripper(self):
        return _Gripper(self.fci_ip)

    def login(self, headless=True):
        from selenium import webdriver
        from selenium.webdriver.firefox.options import Options
        from selenium.webdriver.common.keys import Keys

        options = Options()
        options.headless = headless

        self.driver = webdriver.Firefox(options=options)
        self.driver.get(f'https://{self.fci_ip}')

        if self.driver.find_elements_by_xpath("//*[contains(text(),'" + 'Warning: Potential Security Risk Ahead' + "')]"):
            print('error')

        elem = self.driver.find_element_by_xpath('//input[@id=(//label[text()="Username"]/@for)]')
        elem.clear()
        elem.send_keys(self.username)

        elem = self.driver.find_element_by_xpath('//input[@id=(//label[text()="Password"]/@for)]')
        elem.clear()
        elem.send_keys(self.password)
        elem.send_keys(Keys.RETURN)

        sleep(0.5)  # [s]
        if self.driver.find_elements_by_xpath("//*[contains(text(),'" + 'Warning: Potential Security Risk Ahead' + "')]"):
            print('error')

        self.logged_in = True

    def unlock_brakes(self):
        if not self.logged_in or self.driver is None:
            return

        elem = self.driver.find_element_by_xpath("//div[contains(@data-bind, 'html: brakesOpen()')]")
        elem.click()

        elem = self.driver.find_element_by_xpath("//button[text()='Open']")
        elem.click()

    def lock_brakes(self):
        if not self.logged_in or self.driver is None:
            return

        sleep(0.5)  # [s]
        elem = self.driver.find_element_by_xpath("//div[contains(@data-bind, 'html: brakesOpen()')]")
        elem.click()
