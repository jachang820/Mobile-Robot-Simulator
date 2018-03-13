from django.shortcuts import render
from django.shortcuts import redirect
from django.http import HttpResponse
from django.views.decorators.csrf import csrf_exempt
import json
import urllib3
import time

# Create your views here.
def index(request):
	# Write to USB through serial
	url = "http://192.168.1.114/36/a/"
	print(url)
	http = urllib3.PoolManager()
	response = http.request('GET', url, retries=None)
	time.sleep(5)
	print(response.data)
	print(response.headers)

	return render(request, 'index.html')

@csrf_exempt
def jsonhandler(request):
	if request.method == 'POST':

		# Write to USB through serial
		url = "http://192.168.1.114/" + request.POST.controls;
		print(url)
		http = urllib3.PoolManager()
		r = http.request('GET', url)

		return JsonResponse(r.data.decode('utf-8'))

	elif request.method == 'GET':
		return redirect('/simulation')