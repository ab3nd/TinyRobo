#!/usr/bin/python

from bs4 import BeautifulSoup as bs
import requests
import re

#Store all the things!
symbolURLs = []

#Set up the initial conditions
prefix = "https://github.com"
suffix = "/KiCad?page=1"

while suffix is not None:
  #Get the page and make soup
  page = requests.get(prefix+suffix)
  soup = bs(page.content)

  #Get all the items like <a href="/KiCad/TO_SOT_Packages_THT.pretty" itemprop="name codeRepository">
  # where it's a link to a .pretty repo
  for link in soup.findAll('a', href=re.compile('\.pretty$'), itemprop="name codeRepository"):
    symbolURLs.append(link['href'])

  #Find out if there's a "next" link, which looks like <a class="next_page" href="/KiCad?page=2" rel="next">Next</a>
  suffix = None
  link = soup.find('a', rel="next")
  if link is None:
    break
  else:
    suffix = link['href']

#Now print out the URLS in the format used by the fp-lib-table file
print "(fp_lib_table"
for item in symbolURLs:
  name = re.search('(\/KiCad\/)([^\.]*)(\.pretty$)', item)
  #print name.group(1, 2, 3)
  print "  (lib (name {0})(type Github)(uri ${{KIGITHUB}}/{1})(options \"\")(descr \"Imported from github\"))".format(name.group(2), name.group(2) + name.group(3))
print ")"
